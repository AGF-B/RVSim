#include <fstream>
#include <iostream>
#include <thread>
#include <ctime>

namespace Status {
	inline constexpr int SUCCESS = 0;
	inline constexpr int INVALID_ARGUMENT = -1;
	inline constexpr int FILE_IO = -2;
}

namespace {
	static void ShowUsage() {
		std::cerr << "---------- RVSim (RISC-V simulator) ----------" << std::endl;
		std::cerr << "rvsim: <firmware image>" << std::endl;
		std::cerr << "  <firmware image>: Binary file containing the firmware image" << std::endl;
	}
}

class RegisterFile {
private:
	uint32_t registers[32] = { 0 };

public:
	uint32_t Read(uint8_t i) const {
		return registers[i];
	}

	void Update(uint8_t i, uint32_t v) {
		if (i != 0) {
			registers[i] = v;
		}
	}
};

class ALU {
private:
	uint32_t AddSub(uint32_t a, uint32_t b, bool sub) {
		return a + (sub ? -b : b);
	}

	uint32_t Comp(uint32_t a, uint32_t b, uint8_t op) {
		static constexpr uint8_t EQ_OP = 0;
		static constexpr uint8_t NEQ_OP = 1;
		static constexpr uint8_t SLE_OP = 4;
		static constexpr uint8_t SGEQ_OP = 5;
		static constexpr uint8_t ULE_OP = 6;
		static constexpr uint8_t UGEQ_OP = 7;

		switch (op) {
		case EQ_OP: return a == b;
		case NEQ_OP: return a != b;
		case SLE_OP: return (int32_t)a < (int32_t)b;
		case SGEQ_OP: return (int32_t)a >= (int32_t)b;
		case ULE_OP: return a < b;
		case UGEQ_OP: return a >= b;
		default: return 0;
		}
	}

	uint32_t Logic(uint32_t a, uint32_t b, uint8_t op) {
		static constexpr uint8_t XOR_OP = 4;
		static constexpr uint8_t OR_OP = 6;
		static constexpr uint8_t AND_OP = 7;

		switch (op) {
		case XOR_OP: return a ^ b;
		case OR_OP: return a | b;
		case AND_OP: return a & b;
		default: return 0;
		}
	}

	uint32_t Shift(uint32_t a, uint32_t b, uint8_t op, bool arithmetic) {
		static constexpr uint8_t SHIFT_LEFT = 1;
		static constexpr uint8_t SHIFT_RIGHT = 5;

		switch (op) {
		case SHIFT_LEFT: return a << b;
		case SHIFT_RIGHT: return arithmetic ? ((a >> b) | -((a & 0x80000000) >> b)) : (a >> b);
		default: return 0;
		}
	}

public:
	uint32_t Execute(uint32_t a, uint32_t b, uint8_t op) {
		static constexpr uint8_t ADDSUB_OP = 0;
		static constexpr uint8_t COMP_OP = 1;
		static constexpr uint8_t LOGIC_OP = 2;
		static constexpr uint8_t SHIFT_OP = 3;

		const uint8_t op_sel = (op >> 4) & 0x3;

		switch (op_sel) {
		case ADDSUB_OP: return AddSub(a, b, op & 8);
		case COMP_OP: return Comp(a, b, op & 7);
		case LOGIC_OP: return Logic(a, b, op & 7);
		case SHIFT_OP: return Shift(a, b, op & 7, op & 8);
		default: return 0;
		}
	}
};

enum class ram_op_t : uint8_t {
	READ32,
	WRITE32,
	SREAD16,
	ZREAD16,
	WRITE16,
	SREAD8,
	ZREAD8,
	WRITE8
};

struct ControlContext {
	uint8_t alu_op;
	uint32_t immediate;
	ram_op_t ram_op;
	uint32_t fast_mem_addr;
};

template<size_t N> uint32_t sign_extend(uint32_t x) {
	union {
		uint32_t uimm : N;
		int32_t imm : N;
	} s;

	s.uimm = x;
	int32_t sign_extended = s.imm;
	return sign_extended;
}

class IR {
private:
	uint32_t count = 0;
	uint32_t Q = 0;

public:
	uint32_t Read() const {
		return Q;
	}

	void Update(uint32_t D) {
		++count;
		Q = D;
	}

	uint32_t QueryCount() const {
		return count;
	}
};

class PC {
private:
	static constexpr uint32_t RESET_VECTOR = 0x80000000;

	uint32_t base = RESET_VECTOR;

	uint32_t PrepareUpdate(bool select_base, bool add_immediate, uint32_t immediate) {
		uint32_t next_base = base;

		if (select_base) {
			next_base -= 4;
		}

		if (add_immediate) {
			next_base += immediate;
		}
		else {
			next_base += 4;
		}

		return next_base;
	}

public:
	uint32_t Read() const {
		return base;
	}

	void Reset() {
		base = RESET_VECTOR;
	}

	void UpdateALU(uint32_t alu_input) {
		base = alu_input & 0xFFFFFFFC;
	}

	void Update(bool select_base, bool add_immediate, uint32_t immediate) {
		base = PrepareUpdate(select_base, add_immediate, immediate);
	}

	uint32_t Peek(bool select_base, bool add_immediate, uint32_t immediate) {
		return PrepareUpdate(select_base, add_immediate, immediate);
	}
};

class MMU {
private:
	virtual uint32_t Read(uint32_t at) const = 0;
	virtual void Write(uint32_t at, uint32_t v) const = 0;

public:
	virtual uint32_t Access(uint32_t at, uint32_t x, ram_op_t op) const = 0;
};

class ControlUnit {
private:
	enum class ControlState : uint_fast32_t {
		FETCH,
		DECODE,
		I_TYPE,
		R_TYPE,
		U_TYPE,
		LOAD,
		S_TYPE,
		BREAK,
		B_TYPE,
		J_TYPE,
		JLR_TP,
		AUIPC,
	};

	static constexpr uint8_t OPCODE_I = 0b0010011;
	static constexpr uint8_t OPCODE_R = 0b0110011;
	static constexpr uint8_t OPCODE_U = 0b0110111;
	static constexpr uint8_t OPCODE_L = 0b0000011;
	static constexpr uint8_t OPCODE_S = 0b0100011;
	static constexpr uint8_t OP_BREAK = 0b1110011;
	static constexpr uint8_t IM_BREAK = 1;
	static constexpr uint8_t F3_BREAK = 0;
	static constexpr uint8_t OPCODE_B = 0b1100011;
	static constexpr uint8_t OPCODE_J = 0b1101111;
	static constexpr uint8_t OPC_JALR = 0b1100111;
	static constexpr uint8_t OP_AUIPC = 0b0010111;

	static constexpr uint8_t F3_LB = 0;
	static constexpr uint8_t F3_LH = 1;
	static constexpr uint8_t F3_LW = 2;
	static constexpr uint8_t F3_LBU = 4;
	static constexpr uint8_t F3_LHU = 5;

	static constexpr uint8_t F3_SB = 0;
	static constexpr uint8_t F3_SH = 1;
	static constexpr uint8_t F3_SW = 2;

	ControlState state = ControlState::FETCH;

	const MMU& mmu_link;
	PC& pc_link;
	IR& ir_link;
	RegisterFile& rf_link;
	ALU& alu_link;

public:
	ControlUnit(const MMU& mmu, PC& pc, IR& ir, RegisterFile& rf, ALU& alu) : mmu_link{mmu}, pc_link {pc}, ir_link{ir}, rf_link{rf}, alu_link{alu} {}

	ControlContext Cycle(uint32_t instruction, bool nreset) {
		const uint8_t opcode = instruction & 0x7F;
		const uint8_t funct3 = (instruction >> 12) & 0x7;
		const uint8_t funct7 = (instruction >> 25) & 0x7F;
		const uint8_t rd = (instruction >> 7) & 0x1F;
		const uint8_t rs1 = (instruction >> 15) & 0x1F;
		const uint8_t rs2 = (instruction >> 20) & 0x1F;
		const uint16_t imm12 = (instruction >> 20) & 0xFFF;
		const uint16_t imm_s_12 =
			(((instruction >> 25) & 0x7F) << 5)
			| ((instruction >> 7) & 0x1F);

		const uint16_t imm_b_13 =
			(((instruction >> 31) & 1) << 12)
			| (((instruction >> 7) & 1) << 11)
			| (((instruction >> 25) & 0x3F) << 5)
			| (((instruction >> 8) & 0xF) << 1)
			| 0;
		const uint32_t imm_u_20 = (instruction >> 12) & 0xFFFFF;
		const uint32_t imm_j_21 =
			(((instruction >> 31) & 1) << 20)
			| (((instruction >> 12) & 0xFF) << 12)
			| (((instruction >> 20) & 1) << 11)
			| (((instruction >> 21) & 0x3FF) << 1)
			| 0;

		ControlContext context = { 0 };
		ControlState next_state = state;

		if (!nreset) {
			pc_link.Reset();
			state = ControlState::FETCH;
			return context;
		}

		switch (state) {
		case ControlState::FETCH: next_state = ControlState::DECODE; break;
		case ControlState::DECODE:
			switch (opcode) {
			case OPCODE_I: next_state = ControlState::I_TYPE; break;
			case OPCODE_R: next_state = ControlState::R_TYPE; break;
			case OPCODE_U: next_state = ControlState::U_TYPE; break;
			case OPCODE_L: next_state = ControlState::LOAD; break;
			case OPCODE_S: next_state = ControlState::S_TYPE; break;
			case OP_BREAK: next_state = ControlState::BREAK; break;
			case OPCODE_B: next_state = ControlState::B_TYPE; break;
			case OPCODE_J: next_state = ControlState::J_TYPE; break;
			case OPC_JALR: next_state = ControlState::JLR_TP; break;
			case OP_AUIPC: next_state = ControlState::AUIPC; break;
			default: next_state = ControlState::BREAK;
			}
			break;
		case ControlState::BREAK: next_state = ControlState::BREAK;	break;
		case ControlState::I_TYPE:
		case ControlState::R_TYPE:
		case ControlState::U_TYPE:
		case ControlState::B_TYPE:
		case ControlState::J_TYPE:
		case ControlState::JLR_TP:
		case ControlState::LOAD:
		case ControlState::S_TYPE:
		case ControlState::AUIPC:
			next_state = ControlState::FETCH;
			break;
		default:
			next_state = ControlState::BREAK;
		}

		uint32_t alu_op = 0;

		switch (opcode) {
		case OPCODE_I:
		case OPCODE_R: {
			if (funct3 == 0) {
				static constexpr uint8_t SUB_F7 = 0x20;
				alu_op = (funct7 == SUB_F7 && opcode == OPCODE_R) ? 0x8 : 0;
			}
			else if (!(funct3 & 0x4) && (funct3 & 0x2)) {
				alu_op = (0x7 << 2) | ((funct3 & 1) << 1) | 0;
			}
			else {
				alu_op =
					(1 << 5)
					| ((~(funct3 >> 1) & funct3 & 1) << 4)
					| ((funct3 == 0x5 && funct7 == 0x20) ? 0x8 : 0)
					| funct3;
			}
			break;
		}
		case OPCODE_L:
		case OPCODE_S:
		case OPC_JALR:
		case OP_AUIPC:
			alu_op = 0;
			break;
		case OPCODE_B:
			alu_op = (0x3 << 3) | funct3;
		}

		switch (state) {
		case ControlState::FETCH:
			ir_link.Update(mmu_link.Access(pc_link.Read(), 0, ram_op_t::READ32));
			pc_link.Update(false, false, 0);
			break;
		case ControlState::DECODE:
			break;
		case ControlState::I_TYPE:
			rf_link.Update(rd, alu_link.Execute(rf_link.Read(rs1), sign_extend<12>(imm12), alu_op));
			break;
		case ControlState::R_TYPE:
			rf_link.Update(rd, alu_link.Execute(rf_link.Read(rs1), rf_link.Read(rs2), alu_op));
			break;
		case ControlState::U_TYPE:
			rf_link.Update(rd, imm_u_20 << 12);
			break;
		case ControlState::LOAD:
			context.immediate = sign_extend<12>(imm12);
			context.fast_mem_addr = context.immediate + rf_link.Read(rs1);
			switch (funct3) {
			case F3_LB: context.ram_op = ram_op_t::SREAD8; break;
			case F3_LH: context.ram_op = ram_op_t::SREAD16; break;
			case F3_LW: context.ram_op = ram_op_t::READ32; break;
			case F3_LBU: context.ram_op = ram_op_t::ZREAD8; break;
			case F3_LHU: context.ram_op = ram_op_t::ZREAD16; break;
			}
			rf_link.Update(rd, mmu_link.Access(context.fast_mem_addr, 0, context.ram_op));
			break;
		case ControlState::S_TYPE:
			context.immediate = sign_extend<12>(imm_s_12);
			context.fast_mem_addr = context.immediate + rf_link.Read(rs1);
			switch (funct3) {
			case F3_SB: context.ram_op = ram_op_t::WRITE8; break;
			case F3_SH: context.ram_op = ram_op_t::WRITE16; break;
			case F3_SW: context.ram_op = ram_op_t::WRITE32; break;
			}
			mmu_link.Access(context.fast_mem_addr, rf_link.Read(rs2), context.ram_op);
			break;
		case ControlState::B_TYPE:
			if (alu_link.Execute(rf_link.Read(rs1), rf_link.Read(rs2), context.alu_op) & 1) {
				pc_link.Update(true, true, sign_extend<13>(imm_b_13));
			}			
			break;
		case ControlState::J_TYPE: {
			uint32_t x = pc_link.Read();
			pc_link.Update(true, true, sign_extend<21>(imm_j_21));
			rf_link.Update(rd, x);
			break;
		}
		case ControlState::JLR_TP: {
			uint32_t x = pc_link.Read();
			pc_link.UpdateALU(alu_link.Execute(rf_link.Read(rs1), sign_extend<12>(imm12), context.alu_op));
			rf_link.Update(rd, x);
			break;
		}
		case ControlState::AUIPC:
			rf_link.Update(rd, pc_link.Peek(true, true, imm_u_20 << 12));
			break;
		}

		state = next_state;
		return context;
	}

	void DumpState() {
		printf("IS: %02x\n", (int)state);
	}
};

class CSR32RO {
private:
	uint32_t value;

public:
	CSR32RO(uint32_t value) : value{value} {}

	uint32_t Write(uint32_t v, bool read) const {
		throw new std::runtime_error("TODO: raise illegal-instruction exception");
	}

	uint32_t Set(uint32_t v, bool overwrite) const {
		uint32_t s = value;

		if (overwrite) {
			throw new std::runtime_error("TODO: raise illegal-instruction exception");
		}

		return s;
	}

	uint32_t Clear(uint32_t v, bool overwrite) const {
		uint32_t s = value;

		if (overwrite) {
			throw new std::runtime_error("TODO: raise illegal-instruction exception");
		}

		return s;
	}
};

class CSR32 {
private:
	const uint32_t ro_mask;
	uint32_t value;

public:
	CSR32(uint32_t ro_mask, uint32_t reset_value) : ro_mask{ro_mask}, value{reset_value} {}

	uint32_t Write(uint32_t v, bool read) {
		uint32_t s = read ? value : 0;

		value = (value & ro_mask) | (v & ~ro_mask);

		return s;
	}

	uint32_t Set(uint32_t v, bool overwrite) {
		uint32_t s = value;

		value = overwrite ? (value | (v & ~ro_mask)) : value;

		return s;
	}

	uint32_t Clear(uint32_t v, bool overwrite) {
		uint32_t s = value;

		value = overwrite ? (value & ~(v & ~ro_mask)) : value;

		return s;
	}
};

class CSRController {
private:
	static constexpr uint32_t vendor_id = 0xB11B111B;
	static constexpr uint32_t arch_id = vendor_id;
	static constexpr uint32_t imp_id = vendor_id;
	static constexpr uint32_t hart_id = 0;
	static constexpr uint32_t config_ptr = 0; // not implemented

	static constexpr uint32_t mstatus_mask = 0xFF93E675;
	static constexpr uint32_t mstatus_reset_value = 0;
	static constexpr uint32_t misa_reset_value = 0x40000100;
	static constexpr uint32_t mie_mask = 0xFFFFF777;
	static constexpr uint32_t mie_reset_value = 0;

	const CSR32RO mvendorid = CSR32RO(vendor_id);
	const CSR32RO marchid = CSR32RO(arch_id);
	const CSR32RO mimpid = CSR32RO(imp_id);
	const CSR32RO mhartid = CSR32RO(hart_id);
	const CSR32RO mconfigptr = CSR32RO(config_ptr);

	CSR32 mstatus = CSR32(mstatus_mask, mstatus_reset_value);
	CSR32RO misa = CSR32RO(misa_reset_value);
	CSR32 mie = CSR32(mie_mask, mie_reset_value);
};

class ExecutionUnit {
private:
	IR ir;

	ControlUnit cu;
	ControlContext context = { 0 };

	RegisterFile rf;

	ALU alu;

	PC pc;

public:
	ExecutionUnit(const MMU& mmu) : cu{mmu, pc, ir, rf, alu} {}

	void Cycle(bool nreset) {
		uint32_t instruction = ir.Read();
		context = cu.Cycle(instruction, nreset);
	}

	void DumpState() {
		for (uint8_t i = 0; i < 32 / 4; ++i) {
			for (uint8_t j = 0; j < 4; ++j) {
				printf("x%02d: %08x ", (uint32_t)(i * 4 + j), rf.Read(i * 4 + j));
			}
			puts("");
		}
		printf("IR: %08x PC: %08x\n", ir.Read(), pc.Read());
		cu.DumpState();
	}

	uint32_t QueryInstructionCounter() const {
		return ir.QueryCount();
	}
};

class CPU {
private:
	const MMU& mmu;

	ExecutionUnit eu;

	uint32_t cycle = 0;
	time_t start_time = time(0);

public:
	CPU(const MMU& mmu) : mmu{mmu}, eu{mmu} {
		eu.Cycle(false);
	};

	void Cycle() {
		eu.Cycle(true);
		++cycle;
	}

	void DumpState() {
		const auto icount = eu.QueryInstructionCounter();
		printf(
			"CPU Cycle: %08x Instruction: %08x CPU Speed: %08d Hz | %08d i/s\n",
			cycle,
			icount,
			(int)(cycle / difftime(time(0), start_time)),
			(int)(icount / difftime(time(0), start_time))
		);
		eu.DumpState();
	}
};

class RAM {
private:
	const size_t limit;
	mutable uint8_t* ram;

public:
	RAM(size_t limit) : limit{limit} {
		ram = new uint8_t[limit]{ 0 };
	}

	uint32_t Read(uint32_t at) const {
		return *(uint32_t*)&ram[at];
	}

	void Write(uint32_t at, uint32_t x) const {
		*(uint32_t*)(&ram[at]) = x;
	}
};

class VROM {
private:
	static constexpr size_t CACHE_SIZE = 0x1000;
	static constexpr size_t CACHED_WORDS = CACHE_SIZE / sizeof(uint32_t);
	std::ifstream& file;

	mutable int32_t cache_owner = -1;
	uint32_t* cache = new uint32_t[CACHED_WORDS]{ 0 };

public:
	VROM(std::ifstream& file) : file{file} {}

	uint32_t Read(uint32_t at) const {
		if (at / CACHE_SIZE != cache_owner) {
			file.clear();
			file.seekg(at - (at % CACHE_SIZE));
			file.read((char*)cache, CACHE_SIZE * sizeof(char));
			cache_owner = at / CACHE_SIZE;
		}
		
		return cache[(at % CACHE_SIZE) / sizeof(uint32_t)];
	}
};

class BasicMMU : public MMU {
// 0x00100000 - 0x00200000: RAM
// 0x80000000 - 0x90000000: Firmware mapping
private:
	static constexpr size_t RAM_LIMIT = 0x100000;
	static constexpr size_t RAM_BASE = 0x100000;

	static constexpr size_t FIRMWARE_LIMIT = 0x10000000;
	static constexpr size_t FIRMWARE_BASE = 0x80000000;

	static constexpr size_t GTEXT_BASE = 0xC000;

	RAM ram;
	VROM firmware;

	virtual uint32_t Read(uint32_t at) const final {
		if (at >= RAM_BASE && at < RAM_BASE + RAM_LIMIT) {
			return ram.Read(at - RAM_BASE);
		}
		else if (at >= FIRMWARE_BASE && at < FIRMWARE_BASE + FIRMWARE_LIMIT) {
			return firmware.Read(at - FIRMWARE_BASE);
		}
		else {
			return 0xFFFFFFFF;
		}
	}
	
	virtual void Write(uint32_t at, uint32_t x) const final {
		if (at >= RAM_BASE && at < RAM_BASE + RAM_LIMIT) {
			ram.Write(at - RAM_BASE, x);
		}
		else if (at == GTEXT_BASE) {
			std::cout << (char)x << std::endl;
		}
	}

public:
	BasicMMU(std::ifstream& firmware) : ram{ RAM_LIMIT }, firmware{ firmware } {}

	uint32_t Access(uint32_t at, uint32_t x, ram_op_t op) const final {
		const uint32_t disp = (at % sizeof(uint32_t));
		const uint32_t disp8 = disp * 8;
		const uint32_t disp16 = (disp8 / 16) * 16;
		const uint32_t effective_at = at - disp;

		const uint32_t rd = Read(effective_at);

		switch (op) {
		case ram_op_t::SREAD8: return sign_extend<8>(rd >> disp8);
		case ram_op_t::ZREAD8: return (rd >> disp8) & 0xFF;
		case ram_op_t::SREAD16: return sign_extend<16>(rd >> disp16);
		case ram_op_t::ZREAD16: return (rd >> disp16) & 0xFFFF;
		case ram_op_t::READ32: return rd;
		case ram_op_t::WRITE8: Write(effective_at, (rd & ~(0xFF << disp8)) | ((x & 0xFF) << disp8)); return rd;
		case ram_op_t::WRITE16: Write(effective_at, (rd & ~(0xFFFF << disp16)) | ((x & 0xFFFF) << disp16)); return rd;
		case ram_op_t::WRITE32: Write(effective_at, x); return rd;
		default: return rd;
		}
	}
};

int main(int argc, char* argv[]) {
	if (argc != 2) {
		ShowUsage();
		return Status::INVALID_ARGUMENT;
	}

	std::ifstream firmware_input(argv[1], std::ios::binary);

	if (!firmware_input.is_open()) {
		std::cerr << "Fatal error: Could not open firmware image " << argv[1] << std::endl;
		return Status::FILE_IO;
	}

	BasicMMU mmu(firmware_input);
	CPU cpu(mmu);

	auto t = std::thread([&]() { while (true) { cpu.Cycle(); } });

	while (true) {
		std::cout << ">>";
		std::string s;
		std::cin >> s;

		if (s == "quit") {
			break;
		}
		else if (s == "dump") {
			cpu.DumpState();
		}
	}

	t.join();

	return Status::SUCCESS;
}
