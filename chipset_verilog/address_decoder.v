//=============================================================================
// 模块名 : address_decoder
// 功能   : M8SBC-486 的地址译码器(存储器 + 板载 IO + ISA 窗口)
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : address_decoder.vhd
//
// 说明   : 所有 *_CS 输出都是低有效。地址由 A23..A2 加 A1、A0 拼成 24 位。
//          x86 的 IO 空间只有 64K，因此 IO 译码只比较低 16 位。
//
// 存储器映射:
//   RAM : 000000-09FFFF (640K)，100000-3FFFFF (3M)，4A0000-4FFFFF (384K 空洞回绕)
//   ROM : 0C8000-0FFFFF (224K 低区)，以及 A31=1 的整个高 2G 区域 (ROM 高 32K)
//
// IO 映射:
//   20-21h PIC / 40-43h PIT / 60h+64h 键盘 / 61h 输出锁存 / 70-71h CMOS
//   其余 IO 以及 0A0000-0C7FFF 存储窗口交给 ISA 总线
//=============================================================================
`timescale 1ns / 1ps

module address_decoder (
    input  wire [23:2] ADDR_IN,        // CPU 地址 A23..A2
    input  wire        ADDR_31,        // CPU 地址 A31
    input  wire        ADDR_A0,        // 字节低位地址
    input  wire        ADDR_A1,        // 地址位 A1
    input  wire        CPU_MIO,        // 0 = IO, 1 = 存储器
    input  wire        CPU_WR,         // 0 = 读, 1 = 写

    input  wire        RAM_CACHEABLE,  // RAM 区域可否缓存 (KEN)
    input  wire        ROM_CACHEABLE,  // ROM 区域可否缓存 (KEN)

    input  wire        INT_ACK,        // 中断响应期间屏蔽地址译码

    output reg         RAM_CS,
    output wire        ROM_CS,         // 只接 ROM 的 OE，因此只允许在读周期有效
    output wire        PIC_CS,
    output wire        PIT_CS,
    output wire        PS2_CS,
    output wire        O61_CS,         // 只写端口 61h (外部用锁存器)
    output wire        ISA_CS,
    output wire        CMOS_CS,

    output wire        OUT_KEN,        // 可缓存指示，低有效
    output wire        OUT_BS16,       // 总线宽度指示(低有效)，本设计恒为 1
    output wire        OUT_BS8         // 8 位总线指示，低有效
);

    //-------------------------------------------------------------------------
    // 24 位线性地址 {A23..A2, A1, A0}
    //-------------------------------------------------------------------------
    wire [23:0] addr = {ADDR_IN, ADDR_A1, ADDR_A0};

    reg  ram_cache;      // 低有效(在 always 块中赋值)
    reg  rom_cache;      // 低有效(在 always 块中赋值)
    reg  rom_cs_i;       // 在 always 块中赋值
    wire pic_cs_i;
    wire pit_cs_i;
    wire ps2_cs_i;
    wire o61_cs_i;
    reg  isa_cs_i;       // 在 always 块中赋值
    wire cmos_cs_i;

    //=========================================================================
    // RAM 片选
    // 仅存储器访问且 A31=0 时有效；地址落在 640K 常规内存、
    // 1M-4M 区段，或 4A0000-4FFFFF(用于回绕访问 384K 内存空洞)时选中。
    //=========================================================================
    always @(*) begin
        RAM_CS    = 1'b1;               // 默认不选中
        ram_cache = 1'b1;
        if ((ADDR_31 == 1'b0) && (CPU_MIO == 1'b1)) begin
            if ((addr < 24'h0A0000) ||
                ((addr >= 24'h100000) && (addr < 24'h400000)) ||
                ((addr >= 24'h4A0000) && (addr < 24'h500000))) begin
                RAM_CS = 1'b0;
                if (RAM_CACHEABLE == 1'b1) begin
                    ram_cache = 1'b0;
                end
            end
        end
    end

    //=========================================================================
    // ROM 片选
    // 低区 0C8000-0FFFFF 共 224K；A31=1 时整段高 2G 都选中，
    // 配合 256K ROM 可以访问到低区看不到的高 32K。
    //=========================================================================
    always @(*) begin
        rom_cs_i  = 1'b1;
        rom_cache = 1'b1;
        if (CPU_MIO == 1'b1) begin
            if (ADDR_31 == 1'b1) begin
                rom_cs_i = 1'b0;
                if (ROM_CACHEABLE == 1'b1) begin
                    rom_cache = 1'b0;
                end
            end
            else if ((addr >= 24'h0C8000) && (addr < 24'h100000)) begin
                rom_cs_i = 1'b0;
                if (ROM_CACHEABLE == 1'b1) begin
                    rom_cache = 1'b0;
                end
            end
        end
    end

    // ROM 只接 OE，因此写周期不允许选中
    assign ROM_CS = (CPU_WR == 1'b0) ? rom_cs_i : 1'b1;

    //=========================================================================
    // 板载 IO 端口译码 (只比较低 16 位，且必须是 IO 访问)
    //=========================================================================
    // PIC：20h - 21h  => A15..A1 = 0000000000010000b
    assign pic_cs_i  = ~((CPU_MIO == 1'b0) && (addr[15:1] == 15'h0010));

    // PIT：40h - 43h  => A15..A2 = 00000000010000b
    assign pit_cs_i  = ~((CPU_MIO == 1'b0) && (addr[15:2] == 14'h0010));

    // 键盘控制器：60h 与 64h
    assign ps2_cs_i  = ~((CPU_MIO == 1'b0) &&
                         ((addr[15:0] == 16'h0060) || (addr[15:0] == 16'h0064)));

    // 输出锁存端口：61h
    assign o61_cs_i  = ~((CPU_MIO == 1'b0) && (addr[15:0] == 16'h0061));

    // CMOS：70h - 71h  => A15..A1 = 000000000111000b
    assign cmos_cs_i = ~((CPU_MIO == 1'b0) && (addr[15:1] == 15'h0038));

    assign PIC_CS  = pic_cs_i;
    assign PIT_CS  = pit_cs_i;
    assign PS2_CS  = ps2_cs_i;
    assign O61_CS  = o61_cs_i;
    assign CMOS_CS = cmos_cs_i;

    //=========================================================================
    // ISA 片选：存储器窗口 0A0000-0C7FFF，以及未被板载设备占用的所有 IO
    //=========================================================================
    always @(*) begin
        if (INT_ACK == 1'b1) begin
            // 没有中断响应时才允许访问 ISA
            isa_cs_i = 1'b1;
            if ((ADDR_31 == 1'b0) && (CPU_MIO == 1'b1)) begin
                if ((addr >= 24'h0A0000) && (addr < 24'h0C8000)) begin
                    isa_cs_i = 1'b0;
                end
            end
            else begin
                // 所有没有被板载设备占用的 IO 访问
                if ((CPU_MIO == 1'b0) &&
                    !((pic_cs_i == 1'b0) || (pit_cs_i == 1'b0) ||
                      (ps2_cs_i == 1'b0) || (o61_cs_i == 1'b0) ||
                      (cmos_cs_i == 1'b0))) begin
                    isa_cs_i = 1'b0;
                end
            end
        end
        else begin
            isa_cs_i = 1'b1;
        end
    end

    assign ISA_CS = isa_cs_i;

    //=========================================================================
    // 总线宽度译码
    // RAM 是 32 位，不需要 BS8/BS16。
    // ROM 及板载 IO 都是 8 位器件；ISA 默认也按 8 位处理(16 位由 ISA 驱动覆盖)。
    //=========================================================================
    assign OUT_BS8 = ~((rom_cs_i  == 1'b0) ||
                       (pic_cs_i  == 1'b0) || (pit_cs_i  == 1'b0) ||
                       (ps2_cs_i  == 1'b0) || (o61_cs_i  == 1'b0) ||
                       (cmos_cs_i == 1'b0) ||
                       (isa_cs_i  == 1'b0));

    assign OUT_BS16 = 1'b1;             // 本设计中没有任何 16 位板载器件

    //=========================================================================
    // 可缓存(KEN)：ROM 或 RAM 任一可缓存时拉低
    //=========================================================================
    assign OUT_KEN = ((rom_cache == 1'b0) || (ram_cache == 1'b0)) ? 1'b0 : 1'b1;

endmodule
