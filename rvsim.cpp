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
	class {
	private:
		uint32_t registers[32] = { 0 };

	public:
		uint32_t Read(uint8_t i) const {
			return registers[i];
		}

		void Write(uint8_t i, uint32_t v) {
			if (i != 0) {
				registers[i] = v;
			}
		}
	} registers;

public:
	uint32_t Read(uint8_t id) {
		return registers.Read(id);
	}

	void Cycle(uint8_t id, bool enable, uint32_t data) {
		if (enable) {
			registers.Write(id, data);
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

struct ControlContext {
	bool branch_op;
	bool ir_enable;
	bool pc_add_immediate;
	bool pc_enable;
	bool pc_select_alu;
	bool pc_select_pc_base;
	bool pc_peek;
	bool rf_write_enable;
	bool select_addr;
	bool select_b;
	bool select_mem;
	bool select_pc;
	bool select_immediate;
	bool select_pc_peek;
	bool write_enable;
	uint8_t alu_op;
	uint32_t immediate;
};

class ControlUnit {
private:
	enum class ControlState {
		FETCH1,
		FETCH2,
		DECODE,
		I_TYPE,
		R_TYPE,
		U_TYPE,
		LOAD1,
		LOAD2,
		S_TYPE,
		BREAK,
		B_TYPE,
		J_TYPE,
		JLR_TP,
		AUIPC1,
		AUIPC2,
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

	ControlState state = ControlState::FETCH1;

	template<size_t N> uint32_t sign_extend(uint32_t x) {
		union {
			uint32_t uimm : N;
			int32_t imm : N;
		} s;

		s.uimm = x;
		int32_t sign_extended = s.imm;
		return sign_extended;
	}

public:
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
			state = ControlState::FETCH1;
			return context;
		}

		switch (state) {
		case ControlState::FETCH1: next_state = ControlState::FETCH2; break;
		case ControlState::FETCH2: next_state = ControlState::DECODE; break;
		case ControlState::DECODE:
			switch (opcode) {
			case OPCODE_I: next_state = ControlState::I_TYPE; break;
			case OPCODE_R: next_state = ControlState::R_TYPE; break;
			case OPCODE_U: next_state = ControlState::U_TYPE; break;
			case OPCODE_L: next_state = ControlState::LOAD1; break;
			case OPCODE_S: next_state = ControlState::S_TYPE; break;
			case OP_BREAK: next_state = ControlState::BREAK; break;
			case OPCODE_B: next_state = ControlState::B_TYPE; break;
			case OPCODE_J: next_state = ControlState::J_TYPE; break;
			case OPC_JALR: next_state = ControlState::JLR_TP; break;
			case OP_AUIPC: next_state = ControlState::AUIPC1; break;
			default: next_state = ControlState::BREAK;
			}
			break;
		case ControlState::BREAK: next_state = ControlState::BREAK;	break;
		case ControlState::LOAD1: next_state = ControlState::LOAD2;	break;
		case ControlState::I_TYPE:
		case ControlState::R_TYPE:
		case ControlState::U_TYPE:
		case ControlState::AUIPC2:
			next_state = ControlState::FETCH2;
			break;
		case ControlState::B_TYPE:
		case ControlState::J_TYPE:
		case ControlState::JLR_TP:
		case ControlState::LOAD2:
		case ControlState::S_TYPE:
			next_state = ControlState::FETCH1;
			break;
		case ControlState::AUIPC1: next_state = ControlState::AUIPC2; break;
		default:
			next_state = ControlState::BREAK;
		}

		switch (state) {
		case ControlState::FETCH1: break;
		case ControlState::FETCH2:
			context.ir_enable = true;
			context.pc_enable = true;
			break;
		case ControlState::DECODE: break;
		case ControlState::I_TYPE: {
			context.immediate = sign_extend<12>(imm12);
			context.rf_write_enable = true;
			break;
		}
		case ControlState::R_TYPE:
			context.rf_write_enable = true;
			context.select_b = true;
			break;
		case ControlState::U_TYPE:
			context.immediate = imm_u_20 << 12;
			context.rf_write_enable = true;
			context.select_immediate = true;
			break;
		case ControlState::LOAD1:
			context.immediate = sign_extend<12>(imm12);
			context.select_addr = true;
			break;
		case ControlState::LOAD2:
			context.rf_write_enable = true;
			context.immediate = sign_extend<12>(imm12);
			context.select_addr = true;
			context.select_mem = true;
			break;
		case ControlState::S_TYPE:
			context.immediate = sign_extend<12>(imm_s_12);
			context.select_addr = true;
			context.write_enable = true;
			break;
		case ControlState::B_TYPE:
			context.immediate = sign_extend<13>(imm_b_13);
			context.branch_op = true;
			context.pc_add_immediate = true;
			context.pc_select_pc_base = true;
			context.select_b = true;
			break;
		case ControlState::J_TYPE:
			context.immediate = sign_extend<21>(imm_j_21);
			context.rf_write_enable = true;
			context.pc_enable = true;
			context.pc_add_immediate = true;
			context.pc_select_pc_base = true;
			context.select_pc = true;
			break;
		case ControlState::JLR_TP:
			context.immediate = sign_extend<12>(imm12);
			context.rf_write_enable = true;
			context.pc_enable = true;
			context.pc_select_alu = true;
			context.select_pc = true;
			break;
		case ControlState::AUIPC1:
			context.immediate = imm_u_20 << 12;
			context.pc_select_pc_base = true;
			context.pc_add_immediate = true;
			context.pc_peek = true;
			break;
		case ControlState::AUIPC2:
			context.rf_write_enable = true;
			context.select_pc = true;
			context.select_pc_peek = true;
		}

		switch (opcode) {
		case OPCODE_I:
		case OPCODE_R: {
			if (funct3 == 0) {
				static constexpr uint8_t SUB_F7 = 0x20;
				context.alu_op = (funct7 == SUB_F7 && opcode == OPCODE_R) ? 0x8 : 0;
			}
			else if (!(funct3 & 0x4) && (funct3 & 0x2)) {
				context.alu_op = (0x7 << 2) | ((funct3 & 1) << 1) | 0;
			}
			else {
				context.alu_op =
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
			context.alu_op = 0;
			break;
		case OPCODE_B:
			context.alu_op = (0x3 << 3) | funct3;
		}

		state = next_state;
		return context;
	}

	void DumpState() {
		printf("IS: %02x\n", (int)state);
	}
};

class IR {
private:
	uint32_t Q = 0;

public:
	uint32_t Read() const {
		return Q;
	}

	void Cycle(bool enable, uint32_t D) {
		if (enable) {
			Q = D;
		}
	}
};

class PC {
private:
	static constexpr uint32_t RESET_VECTOR = 0x80000000;

	uint32_t base = RESET_VECTOR;
	uint32_t peek_base = 0;

public:
	uint32_t Read() const {
		return base;
	}

	uint32_t Peek() const {
		return peek_base;
	}

	void Cycle(bool nreset, ControlContext context, uint32_t immediate, uint32_t alu_input) {
		if (!nreset) {
			base = RESET_VECTOR;
		}
		else if (context.pc_peek || context.pc_enable || (context.branch_op && (alu_input & 1))) {
			uint32_t next_base = base;

			if (context.pc_select_alu) {
				next_base = alu_input & 0xFFFFFFFC;
			}
			else {
				if (context.pc_select_pc_base) {
					next_base -= 4;
				}

				if (context.pc_add_immediate) {
					next_base += immediate;
				}
				else {
					next_base += 4;
				}
			}

			if (context.pc_peek) {
				peek_base = next_base;
			}
			else {
				base = next_base;
			}
		}
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

class MMU {
public:
	virtual uint32_t Read(uint32_t at) const = 0;
	virtual void Write(uint32_t at, uint32_t v) const = 0;
};

struct ExecutionContext {
	uint32_t address;
	uint32_t write_data;
	bool write_enable;
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
	ExecutionContext Cycle(bool nreset, uint32_t rdata) {
		uint32_t instruction = ir.Read();

		// Read registers
		uint32_t ra = rf.Read((instruction >> 15) & 0x1F);
		uint32_t rb = rf.Read((instruction >> 20) & 0x1F);

		// Read next pc
		uint32_t next_pc = context.select_pc_peek ? pc.Peek() : pc.Read();

		// select immediate or register B
		uint32_t sel_b = context.select_b ? rb : context.immediate;

		// execute ALU
		uint32_t alu_res = alu.Execute(ra, sel_b, context.alu_op);

		// select alu result or immediate
		uint32_t alu_mux = context.select_immediate ? context.immediate : alu_res;

		// select next pc or memory data
		uint32_t pc_mux = context.select_mem ? rdata : next_pc;

		// final write back data mux
		uint32_t final_mux = (context.select_pc || context.select_mem) ? pc_mux : alu_mux;

		// memory address mux
		uint32_t write_data = rb;
		bool write_enable = context.write_enable;

		pc.Cycle(nreset, context, context.immediate, alu_res);
		rf.Cycle((instruction >> 7) & 0x1F, context.rf_write_enable, final_mux);
		context = cu.Cycle(instruction, nreset);
		ir.Cycle(context.ir_enable, rdata);

		uint32_t addr_mux = context.select_addr ? alu_res : pc.Read();

		return ExecutionContext{
			.address = addr_mux,
			.write_data = rf.Read((instruction >> 20) & 0x1F),
			.write_enable = context.write_enable
		};
	}

	void DumpState() {
		for (size_t i = 0; i < 32 / 4; ++i) {
			for (size_t j = 0; j < 4; ++j) {
				printf("x%02d: %08x ", (uint32_t)(i * 4 + j), rf.Read(i * 4 + j));
			}
			puts("");
		}
		printf("IR: %08x PC: %08x PE: %01x WE: %01x\n", ir.Read(), pc.Read(), context.pc_enable, context.rf_write_enable);
		cu.DumpState();
	}
};

class CPU {
private:
	const MMU& mmu;

	ExecutionUnit eu;
	ExecutionContext context;

	uint32_t cycle = 0;
	time_t start_time = time(0);

public:
	CPU(const MMU& mmu) : mmu{mmu} {
		context = eu.Cycle(false, 0);
	};

	void Cycle() {
		if (context.write_enable) {
			mmu.Write(context.address, context.write_data);
		}

		uint32_t rdata = mmu.Read(context.address);
		context = eu.Cycle(true, rdata);
		++cycle;
	}

	void DumpState() {
		printf("CPU Cycle: %08x CPU Speed: %08d Hz\n", cycle, (int)(cycle / difftime(time(0), start_time)));
		eu.DumpState();
	}
};

class RAM {
private:
	const size_t limit;
	mutable uint8_t* ram;

public:
	RAM(size_t limit) : limit{limit} {
		ram = new uint8_t[limit];
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

public:
	BasicMMU(std::ifstream& firmware) : ram{RAM_LIMIT}, firmware{firmware} {}

	virtual uint32_t Read(uint32_t at) const final {
		at -= at % 4;

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
		at -= at % 4;

		if (at >= RAM_BASE && at < RAM_BASE + RAM_LIMIT) {
			ram.Write(at - RAM_BASE, x);
		}
		else if (at == GTEXT_BASE) {
			std::cout << (char)x << std::endl;
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
