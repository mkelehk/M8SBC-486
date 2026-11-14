//=============================================================================
// 模块名 : KBC_8042
// 功能   : "够用" 版 8042 键盘控制器 (只支持键盘，不含鼠标)
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : KBC_8042.vhd
//
// 说明   : * CPU 侧：60h 数据口 / 64h 命令状态口
//          * 键盘侧：PS/2 同步串行协议(11 位帧：起始位 + 8 数据位 + 奇校验 + 停止位)
//          * 支持 scan code set 2 -> set 1 翻译(含 0xE0 扩展码与 0xF0 释放码)
//          * 支持标准 8042 命令：20/60/AA/AB/AD/AE/D0/D1/FE
//          * IRQ1 由输出缓冲区满(OBF)与中断使能决定；输出口 bit0/bit1 分别
//            控制 CPU 复位与 A20
//
// 时钟域   : CLK(CPU 时钟) 与 CLK_1_1931M(1.193 MHz) 之间用 2 级同步器握手，
//            双向缓冲区的"可用/已清除"标志均做跨时钟域处理。
//=============================================================================
`timescale 1ns / 1ps

module KBC_8042 (
    input  wire       CLK,           // CPU 时钟，最低 8 MHz
    input  wire       CLK_1_1931M,   // 固定 1.1931 MHz (14.318 MHz / 12)
    input  wire       RST_n,         // 低有效复位

    input  wire       CS_n,          // 片选，低有效
    input  wire       RD_n,          // 读选通，低有效
    input  wire       WR_n,          // 写选通，低有效
    input  wire       A2,            // 0 = 60h 数据口, 1 = 64h 命令/状态口
    input  wire [7:0] DATA_IN,
    output reg  [7:0] DATA_OUT,

    input  wire       PS2_CLK_IN,
    output wire       PS2_CLK_OUT,   // 1 = 应在外围转换为 Z 并上拉
    input  wire       PS2_DAT_IN,
    output wire       PS2_DAT_OUT,   // 1 = 应在外围转换为 Z 并上拉

    output wire       IRQ1,
    output wire       A20_GATE,
    output wire       CPU_RESET_n    // 低有效
);

    //-------------------------------------------------------------------------
    // 命令码
    //-------------------------------------------------------------------------
    localparam [7:0] CMD_READ_CONFIG  = 8'h20;  // 读配置字节
    localparam [7:0] CMD_WRITE_CONFIG = 8'h60;  // 写配置字节
    localparam [7:0] CMD_TEST_CTRL    = 8'hAA;  // 自检
    localparam [7:0] CMD_TEST_PORT    = 8'hAB;  // 测试 PS/2 端口
    localparam [7:0] CMD_DISABLE_PORT = 8'hAD;  // 关闭键盘
    localparam [7:0] CMD_ENABLE_PORT  = 8'hAE;  // 打开键盘
    localparam [7:0] CMD_READ_OPORT   = 8'hD0;  // 读输出口
    localparam [7:0] CMD_WRITE_OPORT  = 8'hD1;  // 写输出口
    localparam [7:0] CMD_PULSE_RESET  = 8'hFE;  // 脉冲复位线

    //-------------------------------------------------------------------------
    // 时间常量(以 1.1931 MHz 时钟计)
    //-------------------------------------------------------------------------
    localparam integer PS2_RECEIVE_TIMEOUT  = 20000;  // 约 20-25 ms
    localparam integer PS2_RESET_PULSE_DUR  = 6000;   // 约 5-6 ms
    localparam integer PS2_SEND_CLK_WAIT    = 120;    // 约 100 us
    localparam integer PS2_SEND_DAT_REL     = 60;     // 释放 DAT 的时刻(必须小于上面)

    //=========================================================================
    // Scan Code Set 2 -> Set 1 翻译表(普通码)
    // 用函数实现，综合成组合逻辑查找表；未定义的码返回 0x00。
    //=========================================================================
    function [7:0] set2_to_set1;
        input [7:0] code;
        begin
            case (code)
                8'h1C: set2_to_set1 = 8'h1E;  // A
                8'h32: set2_to_set1 = 8'h30;  // B
                8'h21: set2_to_set1 = 8'h2E;  // C
                8'h23: set2_to_set1 = 8'h20;  // D
                8'h24: set2_to_set1 = 8'h12;  // E
                8'h2B: set2_to_set1 = 8'h21;  // F
                8'h34: set2_to_set1 = 8'h22;  // G
                8'h33: set2_to_set1 = 8'h23;  // H
                8'h43: set2_to_set1 = 8'h17;  // I
                8'h3B: set2_to_set1 = 8'h24;  // J
                8'h42: set2_to_set1 = 8'h25;  // K
                8'h4B: set2_to_set1 = 8'h26;  // L
                8'h3A: set2_to_set1 = 8'h32;  // M
                8'h31: set2_to_set1 = 8'h31;  // N
                8'h44: set2_to_set1 = 8'h18;  // O
                8'h4D: set2_to_set1 = 8'h19;  // P
                8'h15: set2_to_set1 = 8'h10;  // Q
                8'h2D: set2_to_set1 = 8'h13;  // R
                8'h1B: set2_to_set1 = 8'h1F;  // S
                8'h2C: set2_to_set1 = 8'h14;  // T
                8'h3C: set2_to_set1 = 8'h16;  // U
                8'h2A: set2_to_set1 = 8'h2F;  // V
                8'h1D: set2_to_set1 = 8'h11;  // W
                8'h22: set2_to_set1 = 8'h2D;  // X
                8'h35: set2_to_set1 = 8'h15;  // Y
                8'h1A: set2_to_set1 = 8'h2C;  // Z
                8'h45: set2_to_set1 = 8'h0B;  // 0
                8'h16: set2_to_set1 = 8'h02;  // 1
                8'h1E: set2_to_set1 = 8'h03;  // 2
                8'h26: set2_to_set1 = 8'h04;  // 3
                8'h25: set2_to_set1 = 8'h05;  // 4
                8'h2E: set2_to_set1 = 8'h06;  // 5
                8'h36: set2_to_set1 = 8'h07;  // 6
                8'h3D: set2_to_set1 = 8'h08;  // 7
                8'h3E: set2_to_set1 = 8'h09;  // 8
                8'h46: set2_to_set1 = 8'h0A;  // 9
                8'h0E: set2_to_set1 = 8'h29;  // `
                8'h4E: set2_to_set1 = 8'h0C;  // -
                8'h55: set2_to_set1 = 8'h0D;  // =
                8'h5D: set2_to_set1 = 8'h2B;  // backslash
                8'h66: set2_to_set1 = 8'h0E;  // 退格
                8'h29: set2_to_set1 = 8'h39;  // 空格
                8'h0D: set2_to_set1 = 8'h0F;  // TAB
                8'h58: set2_to_set1 = 8'h3A;  // Caps Lock
                8'h12: set2_to_set1 = 8'h2A;  // 左 Shift
                8'h14: set2_to_set1 = 8'h1D;  // 左 Ctrl
                8'h11: set2_to_set1 = 8'h38;  // 左 Alt
                8'h59: set2_to_set1 = 8'h36;  // 右 Shift
                8'h5A: set2_to_set1 = 8'h1C;  // 回车
                8'h76: set2_to_set1 = 8'h01;  // ESC
                8'h05: set2_to_set1 = 8'h3B;  // F1
                8'h06: set2_to_set1 = 8'h3C;  // F2
                8'h04: set2_to_set1 = 8'h3D;  // F3
                8'h0C: set2_to_set1 = 8'h3E;  // F4
                8'h03: set2_to_set1 = 8'h3F;  // F5
                8'h0B: set2_to_set1 = 8'h40;  // F6
                8'h83: set2_to_set1 = 8'h41;  // F7
                8'h0A: set2_to_set1 = 8'h42;  // F8
                8'h01: set2_to_set1 = 8'h43;  // F9
                8'h09: set2_to_set1 = 8'h44;  // F10
                8'h78: set2_to_set1 = 8'h57;  // F11
                8'h07: set2_to_set1 = 8'h58;  // F12
                8'h7E: set2_to_set1 = 8'h46;  // Scroll Lock
                8'h54: set2_to_set1 = 8'h1A;  // [
                8'h77: set2_to_set1 = 8'h45;  // Num Lock
                8'h7C: set2_to_set1 = 8'h37;  // 小键盘 *
                8'h7B: set2_to_set1 = 8'h4A;  // 小键盘 -
                8'h79: set2_to_set1 = 8'h4E;  // 小键盘 +
                8'h71: set2_to_set1 = 8'h53;  // 小键盘 .
                8'h70: set2_to_set1 = 8'h52;  // 小键盘 0
                8'h69: set2_to_set1 = 8'h4F;  // 小键盘 1
                8'h72: set2_to_set1 = 8'h50;  // 小键盘 2
                8'h7A: set2_to_set1 = 8'h51;  // 小键盘 3
                8'h6B: set2_to_set1 = 8'h4B;  // 小键盘 4
                8'h73: set2_to_set1 = 8'h4C;  // 小键盘 5
                8'h74: set2_to_set1 = 8'h4D;  // 小键盘 6
                8'h6C: set2_to_set1 = 8'h47;  // 小键盘 7
                8'h75: set2_to_set1 = 8'h48;  // 小键盘 8
                8'h7D: set2_to_set1 = 8'h49;  // 小键盘 9
                8'h5B: set2_to_set1 = 8'h1B;  // ]
                8'h4C: set2_to_set1 = 8'h27;  // ;
                8'h52: set2_to_set1 = 8'h28;  // '
                8'h41: set2_to_set1 = 8'h33;  // ,
                8'h49: set2_to_set1 = 8'h34;  // .
                8'h4A: set2_to_set1 = 8'h35;  // /
                8'hAB: set2_to_set1 = 8'hAB;  // 设备 ID 字节 1
                default: set2_to_set1 = 8'h00;
            endcase
        end
    endfunction

    //=========================================================================
    // 0xE0 扩展码翻译表
    //=========================================================================
    function [7:0] set2_to_set1_ext;
        input [7:0] code;
        begin
            case (code)
                8'h1F: set2_to_set1_ext = 8'h5B;  // 左 Win
                8'h14: set2_to_set1_ext = 8'h1D;  // 右 Ctrl
                8'h27: set2_to_set1_ext = 8'h5C;  // 右 Win
                8'h11: set2_to_set1_ext = 8'h38;  // 右 Alt
                8'h2F: set2_to_set1_ext = 8'h5D;  // 应用键
                8'h70: set2_to_set1_ext = 8'h52;  // Insert
                8'h6C: set2_to_set1_ext = 8'h47;  // Home
                8'h7D: set2_to_set1_ext = 8'h49;  // Page Up
                8'h71: set2_to_set1_ext = 8'h53;  // Delete
                8'h69: set2_to_set1_ext = 8'h4F;  // End
                8'h7A: set2_to_set1_ext = 8'h51;  // Page Down
                8'h75: set2_to_set1_ext = 8'h48;  // 上箭头
                8'h6B: set2_to_set1_ext = 8'h4B;  // 左箭头
                8'h72: set2_to_set1_ext = 8'h50;  // 下箭头
                8'h74: set2_to_set1_ext = 8'h4D;  // 右箭头
                8'h4A: set2_to_set1_ext = 8'h35;  // 小键盘 /
                8'h5A: set2_to_set1_ext = 8'h1C;  // 小键盘 回车
                8'h12: set2_to_set1_ext = 8'h2A;  // 伪左 Shift (方向/小键盘区)
                8'h59: set2_to_set1_ext = 8'h36;  // 伪右 Shift
                8'h7C: set2_to_set1_ext = 8'h37;  // Print Screen
                default: set2_to_set1_ext = 8'h00;
            endcase
        end
    endfunction

    //=========================================================================
    // 状态寄存器位
    //=========================================================================
    reg status_obf = 1'b0;   // 输出缓冲区满 (0 = 空, 1 = 满)
    reg status_ibf = 1'b0;   // 输入缓冲区满
    reg status_sysf = 1'b0;  // 系统标志(复位后清 0，由固件置位)
    reg status_cd = 1'b0;    // 0 = 数据写入 PS/2, 1 = 命令写入控制器
    reg status_terr = 1'b0;  // 超时错误
    reg status_perr = 1'b0;  // 奇偶校验错误

    reg status_sysf_f1 = 1'b0;      // 来自 PS/2 域的 2 级同步
    reg status_sysf_f2 = 1'b0;

    // status_cd 在 CPU 域(CLK)写入、在 PS/2 域(CLK_1_1931M)被判读，
    // 原 VHDL 没有做同步，这里补 2 级同步器消除亚稳态。
    // 它和 pass_to_ps2_cmd_av 的同步级数相同，两者延迟一致，
    // 因此 PS/2 域看到的"命令/数据"属性与"数据有效"是同一时刻的快照。
    reg status_cd_f1 = 1'b0;
    reg status_cd_f2 = 1'b0;
    reg interrupt_en   = 1'b0;
    reg interrupt_en_f1 = 1'b0;
    reg interrupt_en_f2 = 1'b0;

    reg [7:0] host_read_buf = 8'h00;
    reg last_rd = 1'b0;
    reg last_rd_clear_ok = 1'b0;

    //-------------------------------------------------------------------------
    // 主机 -> 控制器 缓冲区
    //-------------------------------------------------------------------------
    reg [7:0] pass_to_ps2_cmd = 8'h00;
    reg [7:0] pass_to_host_data = 8'h00;   // 控制器 -> 主机 缓冲区
    reg       pass_to_host_perr = 1'b0;
    reg       pass_to_host_terr = 1'b0;

    reg pass_to_ps2_cmd_av    = 1'b0;      // 命令缓冲区有数据
    reg pass_to_ps2_cmd_av_1  = 1'b0;
    reg pass_to_ps2_cmd_av_2  = 1'b0;

    reg pass_to_ps2_cmd_cl    = 1'b0;      // 清除"有数据"标志
    reg pass_to_ps2_cmd_cl_1  = 1'b0;
    reg pass_to_ps2_cmd_cl_2  = 1'b0;

    reg pass_to_ps2_cmd_av_rdy   = 1'b0;   // 回传到 CPU 域，表示已被接收
    reg pass_to_ps2_cmd_av_rdy_1 = 1'b0;
    reg pass_to_ps2_cmd_av_rdy_2 = 1'b0;

    reg pass_to_host_data_av   = 1'b0;     // 0x60 有数据可读
    reg pass_to_host_data_av_1 = 1'b0;
    reg pass_to_host_data_av_2 = 1'b0;
    reg pass_to_host_data_av_3 = 1'b0;

    reg pass_to_host_data_cl   = 1'b0;     // 清除上面的标志
    reg pass_to_host_data_cl_1 = 1'b0;
    reg pass_to_host_data_cl_2 = 1'b0;

    reg pass_to_host_data_cl_ack   = 1'b0; // 慢时钟域对清除的应答
    reg pass_to_host_data_cl_ack_1 = 1'b0;
    reg pass_to_host_data_cl_ack_2 = 1'b0;

    //-------------------------------------------------------------------------
    // PS/2 物理接口
    //-------------------------------------------------------------------------
    reg PS2_CLK_IN_1 = 1'b0;
    reg PS2_CLK_IN_2 = 1'b0;
    reg PS2_CLK_IN_3 = 1'b0;
    reg PS2_DAT_IN_1 = 1'b0;
    reg PS2_DAT_IN_2 = 1'b0;

    reg PS2_CLK_OUT_buf = 1'b1;
    reg PS2_DAT_OUT_buf = 1'b1;

    reg [3:0]  ps2_bit_count    = 4'd0;    // 当前位序号
    reg [7:0]  ps2_receive_data = 8'h00;   // 接收移位寄存器
    reg        ps2_parity_bit   = 1'b0;
    reg [14:0] ps2_timeout_cnt  = 15'd0;   // 接收/发送超时计数

    //-------------------------------------------------------------------------
    // 控制器内部
    //-------------------------------------------------------------------------
    reg [7:0] ps2_cmd_buf = 8'h00;         // 当前待处理的数据/命令

    localparam [1:0] WRITE_PS2  = 2'd0;
    localparam [1:0] WRITE_CFG  = 2'd1;
    localparam [1:0] WRITE_OPORT = 2'd2;
    reg [1:0] PS2_DATA_TARGET = WRITE_PS2;

    reg ps2_cfg_interrupt  = 1'b1;         // 配置字节 bit0
    reg ps2_cfg_sysf       = 1'b0;         // 配置字节 bit2
    reg ps2_cfg_port_clock = 1'b0;         // 配置字节 bit4 (0 = 端口使能)
    reg ps2_cfg_translation = 1'b0;        // 配置字节 bit6

    reg [7:0] ps2_output_port = 8'b11000001;  // CPU 不复位，时钟线释放

    reg [12:0] ps2_reset_pulse_count  = 13'd0;
    reg        ps2_reset_pulse_active = 1'b0;
    reg        ps2_reset_pulse        = 1'b0;
    reg        ps2_reset_trigger      = 1'b0;
    reg        ps2_reset_trigger_last = 1'b0;

    reg ps2_translation_ext   = 1'b0;
    reg ps2_translation_break = 1'b0;

    reg        ps2_send_clk_waiting = 1'b0;
    reg [6:0]  ps2_send_clk_waiting_count = 7'd0;

    localparam [2:0] ST_IDLE           = 3'd0;
    localparam [2:0] ST_CMD_REC        = 3'd1;
    localparam [2:0] ST_DATA_REC       = 3'd2;
    localparam [2:0] ST_WAIT_CMD_CLEAR = 3'd3;
    localparam [2:0] ST_PS2_REC        = 3'd4;
    localparam [2:0] ST_PS2_SEND       = 3'd5;
    localparam [2:0] ST_WAIT_OBF_ACK   = 3'd6;
    reg [2:0] PS2_STATE = ST_IDLE;

    // 原 VHDL 中的 variable，这里提到模块级
    reg       data_parity_tmp;
    reg [7:0] translated_data_v;

    //=========================================================================
    // CPU 接口 (CLK 域)
    //=========================================================================
    always @(posedge CLK or negedge RST_n) begin
        if (RST_n == 1'b0) begin
            status_obf             <= 1'b0;
            status_ibf             <= 1'b0;
            status_sysf            <= 1'b0;
            status_cd              <= 1'b0;
            status_terr            <= 1'b0;
            status_perr            <= 1'b0;
            status_sysf_f2         <= 1'b0;
            interrupt_en           <= 1'b0;
            interrupt_en_f2        <= 1'b0;
            host_read_buf          <= 8'h00;
            last_rd                <= 1'b0;
            last_rd_clear_ok       <= 1'b0;
            pass_to_ps2_cmd        <= 8'h00;
            pass_to_ps2_cmd_av     <= 1'b0;
            pass_to_ps2_cmd_cl_1   <= 1'b0;
            pass_to_ps2_cmd_cl_2   <= 1'b0;
            pass_to_ps2_cmd_av_rdy_1 <= 1'b0;
            pass_to_ps2_cmd_av_rdy_2 <= 1'b0;
            pass_to_host_data_av_1 <= 1'b0;
            pass_to_host_data_av_2 <= 1'b0;
            pass_to_host_data_av_3 <= 1'b0;
            pass_to_host_data_cl   <= 1'b0;
            pass_to_host_data_cl_ack_1 <= 1'b0;
            pass_to_host_data_cl_ack_2 <= 1'b0;
        end
        else begin
            //-------------------------------------------------------------
            // 跨时钟域同步
            //-------------------------------------------------------------
            pass_to_ps2_cmd_cl_1 <= pass_to_ps2_cmd_cl;
            pass_to_ps2_cmd_cl_2 <= pass_to_ps2_cmd_cl_1;

            pass_to_ps2_cmd_av_rdy_1 <= pass_to_ps2_cmd_av_rdy;
            pass_to_ps2_cmd_av_rdy_2 <= pass_to_ps2_cmd_av_rdy_1;

            pass_to_host_data_cl_ack_1 <= pass_to_host_data_cl_ack;
            pass_to_host_data_cl_ack_2 <= pass_to_host_data_cl_ack_1;

            status_sysf_f2  <= status_sysf_f1;
            status_sysf     <= status_sysf_f2;
            interrupt_en_f2 <= interrupt_en_f1;
            interrupt_en    <= interrupt_en_f2;

            // 对端已确认取走数据 => 撤销"有数据"
            if ((pass_to_ps2_cmd_cl_2 == 1'b1) && (pass_to_ps2_cmd_av == 1'b1)) begin
                pass_to_ps2_cmd_av <= 1'b0;
            end

            // 输入缓冲区满标志
            if ((pass_to_ps2_cmd_av == 1'b1) ||
                (pass_to_ps2_cmd_av_rdy_2 == 1'b1) ||
                (pass_to_ps2_cmd_cl_2 == 1'b1)) begin
                status_ibf <= 1'b1;
            end
            else if ((status_ibf == 1'b1) && (pass_to_host_data_cl_ack_2 == 1'b0)) begin
                status_ibf <= 1'b0;
            end

            //-------------------------------------------------------------
            // CPU 写：60h = 数据，64h = 命令
            //-------------------------------------------------------------
            if ((CS_n == 1'b0) && (WR_n == 1'b0)) begin
                if (A2 == 1'b0) begin
                    if (status_ibf == 1'b0) begin
                        pass_to_ps2_cmd    <= DATA_IN;
                        pass_to_ps2_cmd_av <= 1'b1;
                        status_cd          <= 1'b0;   // 数据
                    end
                end
                else begin
                    if (status_ibf == 1'b0) begin
                        pass_to_ps2_cmd    <= DATA_IN;
                        pass_to_ps2_cmd_av <= 1'b1;
                        status_cd          <= 1'b1;   // 命令
                    end
                end
            end

            //-------------------------------------------------------------
            // 从 PS/2 域收到一个字节
            //-------------------------------------------------------------
            pass_to_host_data_av_1 <= pass_to_host_data_av;
            pass_to_host_data_av_2 <= pass_to_host_data_av_1;
            pass_to_host_data_av_3 <= pass_to_host_data_av_2;

            if ((pass_to_host_data_av_2 == 1'b1) && (pass_to_host_data_av_3 == 1'b0)) begin
                // 上升沿 => 数据已到，锁存并置 OBF
                host_read_buf       <= pass_to_host_data;
                status_terr         <= pass_to_host_terr;
                status_perr         <= pass_to_host_perr;
                status_obf          <= 1'b1;
                pass_to_host_data_cl <= 1'b1;
            end
            else if ((CS_n == 1'b0) && (RD_n == 1'b0) && (A2 == 1'b0) &&
                     (status_obf == 1'b1)) begin
                // CPU 正在读 60h
                last_rd <= 1'b1;
            end
            else begin
                if (last_rd == 1'b1) begin
                    // 读结束：清 OBF，但保留 host_read_buf(它是锁存器)
                    last_rd           <= 1'b0;
                    status_obf        <= 1'b0;
                    last_rd_clear_ok  <= 1'b1;
                end
                if ((last_rd_clear_ok == 1'b1) && (last_rd == 1'b0)) begin
                    if (pass_to_host_data_cl_ack_2 == 1'b1) begin
                        pass_to_host_data_cl  <= 1'b0;
                        last_rd_clear_ok      <= 1'b0;
                    end
                end
            end
        end
    end

    //=========================================================================
    // CPU 读 (组合)
    // 60h 读出最近一次的数据；即使 OBF=0 也返回上次的值，
    // 因为一些 TSR(如新版 MS-DOS 的 EDIT)会在 BIOS 读完后再次读 60h。
    //=========================================================================
    always @(*) begin
        DATA_OUT = 8'h00;
        if (RST_n == 1'b1) begin
            if ((CS_n == 1'b0) && (RD_n == 1'b0) && (WR_n == 1'b1)) begin
                if (A2 == 1'b0) begin
                    DATA_OUT = host_read_buf;
                end
                else begin
                    // 状态寄存器：bit4 恒为 1 (键盘未锁定)
                    DATA_OUT = {status_perr, status_terr, 1'b0, 1'b1,
                                status_cd, status_sysf, status_ibf, status_obf};
                end
            end
        end
    end

    assign IRQ1        = status_obf && interrupt_en;   // CPU 时钟域
    assign A20_GATE    = ps2_output_port[1];           // PS/2 时钟域
    assign CPU_RESET_n = ps2_output_port[0] && !ps2_reset_pulse;

    assign PS2_CLK_OUT = PS2_CLK_OUT_buf;
    assign PS2_DAT_OUT = PS2_DAT_OUT_buf;

    //=========================================================================
    // 复位脉冲发生器 (1.1931 MHz 域)
    //=========================================================================
    always @(posedge CLK_1_1931M) begin
        if (ps2_reset_trigger != ps2_reset_trigger_last) begin
            ps2_reset_trigger_last <= ps2_reset_trigger;
            ps2_reset_pulse_active <= 1'b1;
            ps2_reset_pulse_count  <= 13'd0;
            ps2_reset_pulse        <= 1'b1;
        end
        else begin
            if (ps2_reset_pulse_count >= PS2_RESET_PULSE_DUR) begin
                ps2_reset_pulse        <= 1'b0;
                ps2_reset_pulse_active <= 1'b0;
            end
            else if (ps2_reset_pulse_active == 1'b1) begin
                ps2_reset_pulse_count <= ps2_reset_pulse_count + 13'd1;
            end
        end
    end

    //=========================================================================
    // PS/2 控制器状态机 (1.1931 MHz 域)
    //=========================================================================
    always @(posedge CLK_1_1931M or negedge RST_n) begin
        if (RST_n == 1'b0) begin
            PS2_STATE              <= ST_IDLE;
            PS2_DATA_TARGET        <= WRITE_PS2;
            PS2_CLK_OUT_buf        <= 1'b0;   // 禁止 PS/2 通信
            PS2_DAT_OUT_buf        <= 1'b1;   // 默认高阻

            pass_to_host_data      <= 8'h00;
            pass_to_host_perr      <= 1'b0;
            pass_to_host_terr      <= 1'b0;
            pass_to_ps2_cmd_av_1   <= 1'b0;
            pass_to_ps2_cmd_av_2   <= 1'b0;
            pass_to_ps2_cmd_cl     <= 1'b0;
            pass_to_ps2_cmd_av_rdy <= 1'b0;
            pass_to_host_data_av   <= 1'b0;
            pass_to_host_data_cl_1 <= 1'b0;
            pass_to_host_data_cl_2 <= 1'b0;
            pass_to_host_data_cl_ack <= 1'b0;

            PS2_CLK_IN_1           <= 1'b0;
            PS2_CLK_IN_2           <= 1'b0;
            PS2_CLK_IN_3           <= 1'b0;
            PS2_DAT_IN_1           <= 1'b0;
            PS2_DAT_IN_2           <= 1'b0;

            ps2_bit_count          <= 4'd0;
            ps2_receive_data       <= 8'h00;
            ps2_parity_bit         <= 1'b0;
            ps2_timeout_cnt        <= 15'd0;

            ps2_cmd_buf            <= 8'h00;
            ps2_cfg_interrupt      <= 1'b1;
            ps2_cfg_sysf           <= 1'b0;
            ps2_cfg_port_clock     <= 1'b0;
            ps2_cfg_translation    <= 1'b0;

            ps2_translation_ext    <= 1'b0;
            ps2_translation_break  <= 1'b0;
            ps2_send_clk_waiting   <= 1'b0;
            ps2_send_clk_waiting_count <= 7'd0;
            status_cd_f1           <= 1'b0;
            status_cd_f2           <= 1'b0;
        end
        else begin
            //-------------------------------------------------------------
            // 跨时钟域同步：命令/数据属性(CPU 域 -> PS/2 域)
            //-------------------------------------------------------------
            status_cd_f1 <= status_cd;
            status_cd_f2 <= status_cd_f1;

            //-------------------------------------------------------------
            // PS/2 输入同步
            // 输出口 bit6/bit7 为 0 时表示该引脚被控制器主动拉低(回环旁路)
            //-------------------------------------------------------------
            PS2_CLK_IN_1 <= PS2_CLK_IN;
            if (ps2_output_port[6] == 1'b1) begin
                PS2_CLK_IN_2 <= PS2_CLK_IN_1;
            end
            else begin
                PS2_CLK_IN_2 <= 1'b0;
            end
            PS2_CLK_IN_3 <= PS2_CLK_IN_2;

            PS2_DAT_IN_1 <= PS2_DAT_IN;
            if (ps2_output_port[7] == 1'b1) begin
                PS2_DAT_IN_2 <= PS2_DAT_IN_1;
            end
            else begin
                PS2_DAT_IN_2 <= 1'b0;
            end

            //-------------------------------------------------------------
            // 数据清除握手(两个方向)
            //-------------------------------------------------------------
            pass_to_host_data_cl_1 <= pass_to_host_data_cl;
            pass_to_host_data_cl_2 <= pass_to_host_data_cl_1;
            pass_to_host_data_cl_ack <= pass_to_host_data_cl_2;

            if (pass_to_host_data_cl_2 == 1'b1) begin
                pass_to_host_data_av <= 1'b0;
            end

            pass_to_ps2_cmd_av_1 <= pass_to_ps2_cmd_av;
            pass_to_ps2_cmd_av_2 <= pass_to_ps2_cmd_av_1;
            pass_to_ps2_cmd_av_rdy <= pass_to_ps2_cmd_av_2;

            case (PS2_STATE)

                //=========================================================
                // 空闲：接收 CPU 命令，或等待 PS/2 起始位
                //=========================================================
                ST_IDLE: begin
                    if ((pass_to_ps2_cmd_av_2 == 1'b1) &&
                        (pass_to_host_data_cl_2 == 1'b0)) begin
                        // 有命令且输出缓冲区空
                        PS2_CLK_OUT_buf    <= 1'b0;   // 占用总线
                        ps2_cmd_buf        <= pass_to_ps2_cmd;
                        pass_to_ps2_cmd_cl <= 1'b1;   // 应答：让 CPU 域的 IBF 清零

                        if (status_cd_f2 == 1'b0) begin
                            PS2_STATE <= ST_DATA_REC;   // 数据 -> 发给键盘
                        end
                        else begin
                            PS2_STATE <= ST_CMD_REC;
                        end
                    end
                    else begin
                        // 没有命令，或输出缓冲区还满着 => 不确认
                        pass_to_ps2_cmd_cl <= 1'b0;

                        if (pass_to_host_data_cl_2 == 1'b0) begin
                            // 端口使能时释放时钟线
                            if (ps2_cfg_port_clock == 1'b0) begin
                                PS2_CLK_OUT_buf <= 1'b1;
                            end
                            else begin
                                PS2_CLK_OUT_buf <= 1'b0;
                            end

                            // 只在端口使能时响应下降沿，避免使能/禁止时的毛刺
                            if ((PS2_CLK_IN_2 == 1'b0) && (PS2_CLK_IN_3 == 1'b1) &&
                                (ps2_cfg_port_clock == 1'b0)) begin
                                if (PS2_DAT_IN_2 == 1'b0) begin   // 起始位
                                    PS2_STATE      <= ST_PS2_REC;
                                    ps2_bit_count  <= 4'd0;
                                    ps2_timeout_cnt <= 15'd0;
                                end
                            end
                        end
                        else begin
                            PS2_CLK_OUT_buf <= 1'b0;
                        end
                    end

                    PS2_DAT_OUT_buf <= 1'b1;   // 空闲时释放数据线
                end

                //=========================================================
                // 处理 64h 写入的控制器命令
                //=========================================================
                ST_CMD_REC: begin
                    case (ps2_cmd_buf)
                        CMD_READ_CONFIG: begin
                            // 返回配置字节
                            pass_to_host_data <= {1'b0, ps2_cfg_translation, 1'b0,
                                                  ps2_cfg_port_clock, 1'b0,
                                                  ps2_cfg_sysf, 1'b0,
                                                  ps2_cfg_interrupt};
                            pass_to_host_perr <= 1'b0;
                            pass_to_host_terr <= 1'b0;
                            pass_to_host_data_av <= 1'b1;
                        end

                        CMD_WRITE_CONFIG: begin
                            PS2_DATA_TARGET <= WRITE_CFG;   // 下一个字节是配置值
                        end

                        CMD_TEST_CTRL: begin
                            pass_to_host_data <= 8'h55;     // 自检通过
                            pass_to_host_perr <= 1'b0;
                            pass_to_host_terr <= 1'b0;
                            pass_to_host_data_av <= 1'b1;
                            ps2_cfg_port_clock <= 1'b1;     // 自检后关闭键盘
                        end

                        CMD_TEST_PORT: begin
                            // TODO: 未做真正的端口测试
                            pass_to_host_data <= 8'h00;     // 无故障
                            pass_to_host_perr <= 1'b0;
                            pass_to_host_terr <= 1'b0;
                            pass_to_host_data_av <= 1'b1;
                        end

                        CMD_DISABLE_PORT: begin
                            ps2_cfg_port_clock <= 1'b1;     // 配置字节 bit4
                        end

                        CMD_ENABLE_PORT: begin
                            // 也可通过向键盘发送数据来打开端口(未公开行为)
                            ps2_cfg_port_clock <= 1'b0;
                        end

                        CMD_READ_OPORT: begin
                            pass_to_host_data <= ps2_output_port;
                            pass_to_host_perr <= 1'b0;
                            pass_to_host_terr <= 1'b0;
                            pass_to_host_data_av <= 1'b1;
                        end

                        CMD_WRITE_OPORT: begin
                            PS2_DATA_TARGET <= WRITE_OPORT; // 下一个字节是输出口值
                        end

                        CMD_PULSE_RESET: begin
                            ps2_reset_trigger <= ~ps2_reset_trigger;
                        end

                        default: begin
                        end
                    endcase

                    ps2_cmd_buf <= 8'h00;
                    PS2_STATE   <= ST_WAIT_CMD_CLEAR;
                end

                //=========================================================
                // 处理 60h 写入的数据
                //=========================================================
                ST_DATA_REC: begin
                    case (PS2_DATA_TARGET)
                        WRITE_PS2: begin
                            // 开始向键盘发送：先拉低时钟，数据保持高
                            ps2_cfg_port_clock  <= 1'b0;  // 写键盘即打开端口
                            PS2_CLK_OUT_buf     <= 1'b0;
                            PS2_DAT_OUT_buf     <= 1'b1;
                            ps2_send_clk_waiting      <= 1'b1;
                            ps2_send_clk_waiting_count <= 7'd0;
                            ps2_timeout_cnt     <= 15'd0;
                            ps2_bit_count       <= 4'd0;
                            PS2_STATE           <= ST_PS2_SEND;
                        end

                        WRITE_CFG: begin
                            ps2_cfg_translation <= ps2_cmd_buf[6];
                            ps2_cfg_port_clock  <= ps2_cmd_buf[4];
                            ps2_cfg_sysf        <= ps2_cmd_buf[2];
                            status_sysf_f1      <= ps2_cmd_buf[2];  // 送 CPU 域
                            ps2_cfg_interrupt   <= ps2_cmd_buf[0];
                            interrupt_en_f1     <= ps2_cmd_buf[0];  // OBF 由 CPU 清

                            PS2_DATA_TARGET <= WRITE_PS2;
                            PS2_STATE       <= ST_WAIT_CMD_CLEAR;
                        end

                        WRITE_OPORT: begin
                            ps2_output_port <= ps2_cmd_buf;

                            PS2_DATA_TARGET <= WRITE_PS2;
                            PS2_STATE       <= ST_WAIT_CMD_CLEAR;
                        end

                        default: begin
                            PS2_DATA_TARGET <= WRITE_PS2;
                            PS2_STATE       <= ST_WAIT_CMD_CLEAR;
                        end
                    endcase
                end

                //=========================================================
                // 等 CPU 域确认命令已被取走
                //=========================================================
                ST_WAIT_CMD_CLEAR: begin
                    if (pass_to_ps2_cmd_av_2 == 1'b0) begin
                        PS2_STATE <= ST_IDLE;
                    end
                end

                //=========================================================
                // 接收键盘字节 (11 位帧)
                //=========================================================
                ST_PS2_REC: begin
                    if (ps2_timeout_cnt >= PS2_RECEIVE_TIMEOUT) begin
                        // 接收超时
                        pass_to_host_data <= 8'hFF;
                        pass_to_host_perr <= 1'b0;
                        pass_to_host_terr <= 1'b1;
                        pass_to_host_data_av <= 1'b1;
                        PS2_STATE <= ST_WAIT_OBF_ACK;
                    end
                    else if ((PS2_CLK_IN_2 == 1'b0) && (PS2_CLK_IN_3 == 1'b1)) begin
                        // 时钟下降沿：采一位
                        if (ps2_bit_count < 4'd8) begin
                            ps2_receive_data <= {PS2_DAT_IN_2, ps2_receive_data[7:1]};
                        end
                        else if (ps2_bit_count == 4'd8) begin
                            ps2_parity_bit <= PS2_DAT_IN_2;
                        end
                        else begin
                            // 停止位之后：校验并处理整字节
                            data_parity_tmp = ~(^ps2_receive_data);   // 奇校验

                            if (data_parity_tmp == ps2_parity_bit) begin
                                if (ps2_cfg_translation == 1'b1) begin
                                    if (ps2_receive_data == 8'hE0) begin
                                        // 扩展码前缀，原样上报
                                        ps2_translation_ext <= 1'b1;
                                        pass_to_host_data    <= ps2_receive_data;
                                        pass_to_host_perr    <= 1'b0;
                                        pass_to_host_terr    <= 1'b0;
                                        pass_to_host_data_av <= 1'b1;
                                        PS2_STATE <= ST_WAIT_OBF_ACK;
                                    end
                                    else if (ps2_receive_data == 8'hF0) begin
                                        // 释放前缀：不上报，等待下一个字节并置 bit7
                                        ps2_translation_break <= 1'b1;
                                        PS2_STATE <= ST_IDLE;
                                    end
                                    else begin
                                        // 控制字节原样通过
                                        if ((ps2_receive_data == 8'hFA) ||
                                            (ps2_receive_data == 8'hFE) ||
                                            (ps2_receive_data == 8'hAA) ||
                                            (ps2_receive_data == 8'hEE) ||
                                            (ps2_receive_data == 8'h00) ||
                                            (ps2_receive_data == 8'hFF) ||
                                            (ps2_receive_data == 8'hFC) ||
                                            (ps2_receive_data == 8'hFD)) begin
                                            translated_data_v = ps2_receive_data;
                                        end
                                        else if (ps2_translation_ext == 1'b1) begin
                                            translated_data_v = set2_to_set1_ext(ps2_receive_data);
                                        end
                                        else begin
                                            translated_data_v = set2_to_set1(ps2_receive_data);
                                        end

                                        if (ps2_translation_break == 1'b1) begin
                                            translated_data_v[7] = 1'b1;   // 置释放位
                                        end

                                        pass_to_host_data    <= translated_data_v;
                                        pass_to_host_perr    <= 1'b0;
                                        pass_to_host_terr    <= 1'b0;
                                        pass_to_host_data_av <= 1'b1;
                                        PS2_STATE <= ST_WAIT_OBF_ACK;

                                        // 清翻译标志
                                        ps2_translation_ext   <= 1'b0;
                                        ps2_translation_break <= 1'b0;
                                    end
                                end
                                else begin
                                    // 不翻译，直接上报
                                    pass_to_host_data    <= ps2_receive_data;
                                    pass_to_host_perr    <= 1'b0;
                                    pass_to_host_terr    <= 1'b0;
                                    pass_to_host_data_av <= 1'b1;
                                    PS2_STATE <= ST_WAIT_OBF_ACK;
                                end
                            end
                            else begin
                                // 校验错误
                                pass_to_host_data <= 8'hFF;
                                pass_to_host_perr <= 1'b1;
                                pass_to_host_terr <= 1'b0;
                                pass_to_host_data_av <= 1'b1;
                                PS2_STATE <= ST_WAIT_OBF_ACK;
                            end
                        end

                        ps2_bit_count   <= ps2_bit_count + 4'd1;
                        ps2_timeout_cnt <= 15'd0;    // 有边沿就清超时
                    end
                    else begin
                        ps2_timeout_cnt <= ps2_timeout_cnt + 15'd1;
                    end
                end

                //=========================================================
                // 向键盘发送字节
                //=========================================================
                ST_PS2_SEND: begin
                    // 提前清 IBF，允许 CPU 在发送期间写入下一个字节
                    if (pass_to_ps2_cmd_av_2 == 1'b0) begin
                        pass_to_ps2_cmd_cl <= 1'b0;
                    end

                    if (ps2_timeout_cnt >= PS2_RECEIVE_TIMEOUT) begin
                        PS2_CLK_OUT_buf <= 1'b1;
                        PS2_DAT_OUT_buf <= 1'b1;

                        pass_to_host_data <= 8'hFF;
                        pass_to_host_perr <= 1'b0;
                        pass_to_host_terr <= 1'b1;
                        pass_to_host_data_av <= 1'b1;
                        PS2_STATE <= ST_WAIT_OBF_ACK;
                    end
                    else begin
                        if (ps2_send_clk_waiting == 1'b1) begin
                            // 起始阶段：时钟为低、数据为高，等待 100us
                            if (ps2_send_clk_waiting_count >= PS2_SEND_CLK_WAIT) begin
                                ps2_send_clk_waiting <= 1'b0;
                                PS2_CLK_OUT_buf <= 1'b1;   // 释放时钟，下一拍送 D0
                            end
                            else begin
                                // 在释放时钟之前先拉低数据线，形成起始位
                                if (ps2_send_clk_waiting_count == PS2_SEND_DAT_REL) begin
                                    PS2_DAT_OUT_buf <= 1'b0;
                                end
                                ps2_send_clk_waiting_count <= ps2_send_clk_waiting_count + 7'd1;
                            end
                        end
                        else begin
                            // 实际数据发送，每个下降沿更新一位
                            if ((PS2_CLK_IN_2 == 1'b0) && (PS2_CLK_IN_3 == 1'b1)) begin
                                if (ps2_bit_count < 4'd8) begin
                                    PS2_DAT_OUT_buf <= ps2_cmd_buf[ps2_bit_count];  // 低位先发
                                end
                                else if (ps2_bit_count == 4'd8) begin
                                    // 奇校验位
                                    PS2_DAT_OUT_buf <= ~(^ps2_cmd_buf);
                                end
                                else if (ps2_bit_count == 4'd9) begin
                                    PS2_DAT_OUT_buf <= 1'b1;   // 停止位
                                end
                                else if (ps2_bit_count == 4'd10) begin
                                    // 读键盘的应答位
                                    if (PS2_DAT_IN_2 == 1'b0) begin
                                        PS2_STATE <= ST_IDLE;  // 发送成功
                                    end
                                    else begin
                                        pass_to_host_data <= 8'hFF;
                                        pass_to_host_perr <= 1'b0;
                                        pass_to_host_terr <= 1'b1;
                                        pass_to_host_data_av <= 1'b1;
                                        PS2_STATE <= ST_WAIT_OBF_ACK;
                                    end
                                end

                                ps2_bit_count   <= ps2_bit_count + 4'd1;
                                ps2_timeout_cnt <= 15'd0;
                            end
                            else begin
                                ps2_timeout_cnt <= ps2_timeout_cnt + 15'd1;
                            end
                        end
                    end
                end

                //=========================================================
                // 等 CPU 域确认数据已被取走，才能回到空闲
                // (否则 IBF 尚未清空时可能立刻处理下一条命令)
                //=========================================================
                ST_WAIT_OBF_ACK: begin
                    if (pass_to_host_data_cl_2 == 1'b1) begin
                        PS2_STATE <= ST_IDLE;
                    end
                end

                default: begin
                    PS2_STATE <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
