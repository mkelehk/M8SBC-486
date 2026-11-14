//=============================================================================
// 模块名 : CMOS
// 功能   : 简化版 CMOS RTC / NVRAM (电池保持 + AVR 串行备份)
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : CMOS.vhd
//
// 说明   : * 端口 70h 写索引、71h 读写数据
//          * 0x00/0x02/0x04/0x06/0x07/0x08/0x09/0x32 为时间寄存器(二进制格式)
//          * 0x0A / 0x0B 为状态寄存器 A / B
//          * 0xFC-0xFF 读出 FPGA 版本号
//          * 0x40-0x5F 为 32 字节用户 NVRAM (掉电由 AVR 保存)
//          * 0xFF 写入 0x17 会锁定 CMOS，直到复位
//
// AVR 串行接口(借用 FPGA 的两根配置引脚):
//   上电后 FPGA 等待 ATMega 串行发送 33 字节；CLK 始终由 AVR 驱动，
//   恢复(FPGA 收)：11110101 [byte0] ... [byte31]
//   保存(FPGA 发)：11110101 [byte0] ... [byte31] 10101010
//   数据在时钟上升沿有效，AVR_IO 为开漏双向线。
//
// 时间保持由外部 1.193 MHz PIT 时钟分频得到 1 Hz 节拍。
//=============================================================================
`timescale 1ns / 1ps

module CMOS (
    input  wire        CLK_IN,      // CPU 时钟(总线同步用)
    input  wire [7:0]  DATA_IN,     // 来自 CPU 的写数据
    output reg  [7:0]  DATA_OUT,    // 送到 CPU 的读数据
    input  wire        CMOS_CS,     // 片选，低有效
    input  wire        WR,          // 写选通，低有效
    input  wire        RD,          // 读选通，低有效
    input  wire        A0,          // 0 = 索引口(70h), 1 = 数据口(71h)

    input  wire        CLK_PIT,     // 1.193 MHz，用于走时

    input  wire        AVR_CLK,     // AVR 提供的串行时钟
    inout  wire        AVR_IO,      // 双向数据(开漏)

    input  wire [31:0] FPGA_VER,    // 版本号，供 0xFC-0xFF 读取
    input  wire        RESET        // 高有效
);

    //-------------------------------------------------------------------------
    // 常量
    //-------------------------------------------------------------------------
    localparam integer CYCLES_WAIT_TO_TRANSFER = 100000;  // 停机多久后开始保存
    localparam [20:0]  PIT_DIV_1S              = 21'd1193181;

    //=========================================================================
    // 时间寄存器 (上电初值与 VHDL 一致；Xilinx 器件配置后寄存器默认即为 0)
    //=========================================================================
    reg [20:0] pit_divider = 21'd0;      // PIT 分频计数
    reg        tick_1hz    = 1'b0;       // 1 Hz 单周期脉冲

    reg [7:0]  seconds = 8'd0;           // 0..59
    reg [7:0]  minutes = 8'd0;           // 0..59
    reg [7:0]  hours   = 8'd0;           // 0..23
    reg [7:0]  weekday = 8'd7;           // 1..7 (周日=1)
    reg [7:0]  day     = 8'd1;           // 1..31
    reg [7:0]  month   = 8'd11;          // 1..12
    reg [7:0]  year    = 8'd25;          // 0..99
    reg [7:0]  century = 8'd20;          // 0..99

    reg [5:0]  max_days_in_month;        // 28..31
    wire       is_leap_year;

    //-------------------------------------------------------------------------
    // 索引寄存器与 NVRAM
    //-------------------------------------------------------------------------
    reg [7:0]  current_register = 8'h00;
    reg [7:0]  cmos_registers [0:31];    // 32 字节，索引 = 寄存器地址[4:0]
    reg        cmos_write_protect = 1'b0;

    //-------------------------------------------------------------------------
    // AVR 串行接口
    //-------------------------------------------------------------------------
    reg        transfer_config          = 1'b1;   // 上电后默认处于"接收"方向
    reg        transfer_config_next_end = 1'b0;

    reg        received_config  = 1'b0;
    reg        received_pream   = 1'b0;
    reg [2:0]  pream_count      = 3'd0;

    reg        s1_avr_clk = 1'b0;        // 2 级同步
    reg        s1_avr_din = 1'b0;
    reg        s2_avr_clk = 1'b0;
    reg        s2_avr_din = 1'b0;
    reg        last_avr_clk = 1'b0;      // 边沿检测

    reg [4:0]  config_count = 5'd0;      // 字节计数 0..31
    reg [2:0]  config_c_bit = 3'd0;      // 位计数 0..7
    reg [7:0]  config_temp  = 8'h00;     // 移位寄存器

    reg        avr_out = 1'b0;           // 1 = 释放总线, 0 = 拉低(见 AVR_IO 赋值)

    //-------------------------------------------------------------------------
    // NVRAM 写入仲裁：AVR 恢复 vs CPU 访问
    //-------------------------------------------------------------------------
    reg        cfg_writer_en   = 1'b0;
    reg [4:0]  cfg_writer_addr = 5'd0;
    reg [7:0]  cfg_writer_val;

    reg        bus_writer_en;
    reg [4:0]  bus_writer_addr;
    reg [7:0]  bus_writer_val = 8'h00;

    wire       ram_write_en;
    wire [4:0] ram_write_addr;
    wire [7:0] ram_write_val;
    reg  [7:0] ram_out;                  // 同步读出的 NVRAM 数据

    reg        bus_access;               // 当前总线访问指向配置 RAM
    reg        cmos_in_range;            // 索引落在 0x40-0x5F

    //-------------------------------------------------------------------------
    // 脏数据检测 / 保存状态机
    //-------------------------------------------------------------------------
    reg [16:0] transfer_timer = 17'd0;
    reg        config_data_dirty      = 1'b0;
    reg        last_config_data_dirty = 1'b0;
    reg        config_do_write        = 1'b0;
    reg        transfer_dirty         = 1'b0;
    reg [1:0]  config_write_phase     = 2'd0;
    reg        cfg_skip_clk           = 1'b0;

    // 下面两个变量原为 VHDL 进程内的 variable，这里提到模块级
    reg        avr_out_tmp;
    reg [7:0]  config_temp_var;
    reg [7:0]  data_out_final;

    //=========================================================================
    // AVR 串行通信进程
    // 在 CLK_IN 下降沿处理(与 VHDL 相同)，所有收发都以 AVR 的时钟为基准。
    //=========================================================================
    always @(negedge CLK_IN) begin
        // 2 级同步，跨时钟域
        s1_avr_clk <= AVR_CLK;
        s1_avr_din <= AVR_IO;
        s2_avr_clk <= s1_avr_clk;
        s2_avr_din <= s1_avr_din;

        //-------------------------------------------------------------
        // CPU 写了 CMOS => 数据变脏并重置超时
        //-------------------------------------------------------------
        if ((WR == 1'b0) && (A0 == 1'b1) && (CMOS_CS == 1'b0) &&
            (cmos_in_range == 1'b1) && (cmos_write_protect == 1'b0)) begin
            transfer_timer    <= 17'd0;
            config_data_dirty <= 1'b1;
        end
        else if ((received_config == 1'b1) && (config_do_write == 1'b0)) begin
            if (transfer_timer >= CYCLES_WAIT_TO_TRANSFER) begin
                // 总线安静足够久，且存在未保存的数据 => 启动保存
                if ((config_data_dirty == 1'b1) || (transfer_dirty == 1'b1)) begin
                    config_count             <= 5'd0;
                    config_c_bit             <= 3'd0;
                    config_do_write          <= 1'b1;
                    transfer_dirty           <= 1'b0;
                    transfer_config          <= 1'b1;
                    transfer_config_next_end <= 1'b0;
                    config_write_phase       <= 2'd0;
                    cfg_writer_en            <= 1'b0;
                    config_data_dirty        <= 1'b0;
                    cfg_skip_clk             <= 1'b1;   // 跳过半个时钟，避免从中间开始
                end
            end
            else begin
                if (config_data_dirty != last_config_data_dirty) begin
                    transfer_timer <= 17'd0;            // 有新写入，重新计时
                end
                else begin
                    transfer_timer <= transfer_timer + 17'd1;
                end
            end
        end

        last_config_data_dirty <= config_data_dirty;
        cfg_writer_addr        <= config_count;

        //-------------------------------------------------------------
        // 检测 AVR 时钟上升沿
        //-------------------------------------------------------------
        if ((last_avr_clk != s2_avr_clk) && (s2_avr_clk == 1'b1)) begin

            if (received_config == 1'b0) begin
                //=========================================================
                // 接收方向：先找前导码 0xF5，再收 32 个字节
                //=========================================================
                config_temp <= {config_temp[6:0], s2_avr_din};

                if (received_pream == 1'b1) begin
                    if (config_c_bit == 3'd7) begin
                        // 一个字节收齐。
                        // 注意：这里必须直接取 CONFIG_TEMP，不能在后面再拼 s2_avr_din。
                        // 原因：检测到前导码 0xF5 的那个时钟沿，除了把 RECEIVED_PREAM 置 1，
                        // 同时也已经把这一个数据位挪进了 CONFIG_TEMP。因此等到下一次
                        // CONFIG_C_BIT=7 的写沿，CONFIG_TEMP 里正好是完整的 8 位本拍字节
                        // （bit7..bit0 = b7..b0）。此时再拼一位会把整字节左移 1 位，
                        // 写进 NVRAM 的就是 {b6..b0, 下一字节的 bit7}，是错的。
                        // 这与 VHDL 侧回退后的写法一一对应。
                        config_c_bit   <= 3'd0;
                        cfg_writer_en  <= 1'b1;
                        cfg_writer_val <= config_temp;

                        if (config_count == 5'd31) begin
                            transfer_config_next_end <= 1'b1;   // 收完 32 字节
                        end
                        else begin
                            config_count <= config_count + 5'd1;
                        end
                    end
                    else begin
                        cfg_writer_en <= 1'b0;
                        config_c_bit  <= config_c_bit + 3'd1;

                        if (transfer_config_next_end == 1'b1) begin
                            transfer_config <= 1'b0;            // 转为保存方向
                            received_config <= 1'b1;
                        end
                    end
                end
                else begin
                    // 等待前导码 11110101
                    if (config_temp == 8'hF5) begin
                        received_pream <= 1'b1;
                    end
                    else begin
                        if (pream_count == 3'd7) begin
                            pream_count <= 3'd0;
                        end
                        else begin
                            pream_count <= pream_count + 3'd1;
                        end
                    end
                end
            end
            else begin
                //=========================================================
                // 发送方向：F5 + 32 字节 + AA
                //=========================================================
                if (cfg_skip_clk == 1'b1) begin
                    cfg_skip_clk <= 1'b0;
                end
                else if (config_do_write == 1'b1) begin
                    if (bus_access == 1'b0) begin
                        // 按当前阶段取出要发送的字节
                        case (config_write_phase)
                            2'd0:    config_temp_var = 8'hF5;   // 前导码
                            2'd1:    config_temp_var = ram_out; // 数据
                            default: config_temp_var = 8'hAA;   // 结束码
                        endcase

                        avr_out_tmp = config_temp_var[3'd7 - config_c_bit];  // MSB 先发

                        if (transfer_dirty == 1'b1) begin
                            avr_out <= 1'b0;          // 传输被总线打断 => 保持低电平作废
                        end
                        else begin
                            avr_out <= avr_out_tmp;   // 1 = 释放(等价原设计的 'Z'), 0 = 拉低
                        end

                        if (config_c_bit == 3'd7) begin
                            config_c_bit <= 3'd0;
                            case (config_write_phase)
                                2'd0: begin
                                    config_write_phase <= 2'd1;
                                end
                                2'd1: begin
                                    if (config_count == 5'd31) begin
                                        config_write_phase <= 2'd2;
                                    end
                                    else begin
                                        config_count <= config_count + 5'd1;
                                    end
                                end
                                default: begin
                                    // 结束码发完，回到空闲
                                    transfer_config <= 1'b0;
                                    config_do_write <= 1'b0;
                                end
                            endcase
                        end
                        else begin
                            config_c_bit <= config_c_bit + 3'd1;
                        end
                    end
                    else begin
                        // 发送期间总线要访问 NVRAM => 放弃本次保存
                        transfer_dirty <= 1'b1;
                        avr_out        <= 1'b0;
                    end
                end
            end
        end

        last_avr_clk <= s2_avr_clk;
    end

    // AVR_IO 为开漏：只有"接收完成且需要拉低"时才驱动 0，否则释放
    assign AVR_IO = ((received_config == 1'b1) && (avr_out == 1'b0)) ? 1'b0 : 1'bz;

    //=========================================================================
    // NVRAM 写端口仲裁
    //=========================================================================
    assign ram_write_val  = ((transfer_config == 1'b1) && (bus_access == 1'b0))
                            ? cfg_writer_val  : bus_writer_val;
    assign ram_write_addr = ((transfer_config == 1'b1) && (bus_access == 1'b0))
                            ? cfg_writer_addr : bus_writer_addr;
    assign ram_write_en   = ((transfer_config == 1'b1) && (bus_access == 1'b0))
                            ? cfg_writer_en   : bus_writer_en;

    //=========================================================================
    // 索引范围检测：0x40 - 0x5F 共 32 字节
    //=========================================================================
    always @(*) begin
        cmos_in_range = ((current_register >= 8'd64) &&
                         (current_register <  8'd96)) ? 1'b1 : 1'b0;
    end

    //=========================================================================
    // NVRAM：下降沿写入，读出的数据同样在下降沿寄存(同步读)
    // 上电时由 FPGA 配置过程把 RAM 清 0，与原 VHDL 的显式初值等价。
    //=========================================================================
    always @(negedge CLK_IN) begin
        if (ram_write_en == 1'b1) begin
            cmos_registers[ram_write_addr] <= ram_write_val;
        end
        ram_out <= cmos_registers[ram_write_addr];
    end

    //=========================================================================
    // CPU 侧寄存器写：索引口(70h) / 数据口(71h)
    // 原 VHDL 把这段写在组合进程中的 FALLING_EDGE(CLK_IN) 分支里，
    // 这里拆成独立的时序块，行为一致且便于综合。
    //=========================================================================
    always @(negedge CLK_IN) begin
        if (RESET == 1'b1) begin
            cmos_write_protect <= 1'b0;
        end
        else if ((CMOS_CS == 1'b0) && (WR == 1'b0)) begin
            if (A0 == 1'b0) begin
                // 写索引寄存器
                current_register <= DATA_IN;
            end
            else begin
                // 写数据：0xFF 寄存器为芯片组命令
                if (current_register == 8'hFF) begin
                    if (DATA_IN == 8'h17) begin
                        cmos_write_protect <= 1'b1;   // 锁定 CMOS，直到复位
                    end
                end
                else begin
                    bus_writer_val <= DATA_IN;
                end
            end
        end
    end

    //=========================================================================
    // CPU 读数据 / 总线访问标志(纯组合)
    //=========================================================================
    always @(*) begin
        data_out_final  = 8'h00;
        bus_writer_addr = current_register[4:0];   // NVRAM 基址按 0x20 对齐
        bus_writer_en   = 1'b0;
        bus_access      = 1'b0;

        if ((CMOS_CS == 1'b0) && (RESET == 1'b0)) begin

            if (WR == 1'b0) begin
                //---------------------------------------------------------
                // 写周期：CPU 正在访问 NVRAM 时告诉 AVR 侧不要抢占
                //---------------------------------------------------------
                if ((cmos_in_range == 1'b1) && (received_config == 1'b1) &&
                    (A0 == 1'b1) && (cmos_write_protect == 1'b0)) begin
                    bus_writer_en = 1'b1;
                    bus_access    = 1'b1;
                end
                else begin
                    bus_writer_en = 1'b0;
                    bus_access    = 1'b0;
                end
            end
            else if (RD == 1'b0) begin
                //---------------------------------------------------------
                // 读周期
                //---------------------------------------------------------
                if ((cmos_in_range == 1'b1) && (received_config == 1'b1) &&
                    (A0 == 1'b1)) begin
                    bus_access = 1'b1;
                end
                else begin
                    bus_access = 1'b0;
                end
                bus_writer_en = 1'b0;

                if (A0 == 1'b0) begin
                    // 读索引寄存器本身
                    data_out_final = current_register;
                end
                else begin
                    // 读数据寄存器
                    case (current_register)
                        8'h00:   data_out_final = seconds;      // 秒
                        8'h02:   data_out_final = minutes;      // 分
                        8'h04:   data_out_final = hours;        // 时
                        8'h06:   data_out_final = weekday;      // 星期
                        8'h07:   data_out_final = day;          // 日
                        8'h08:   data_out_final = month;        // 月
                        8'h09:   data_out_final = year;         // 年
                        8'h32:   data_out_final = century;      // 世纪
                        8'h0A: begin
                            // 状态寄存器 A：UIP=0，32.768kHz 基准，1kHz 中断速率
                            data_out_final = 8'b00100110;
                        end
                        8'h0B: begin
                            // 状态寄存器 B：正常运行，二进制模式，24 小时制
                            data_out_final = 8'b00000110;
                        end
                        8'hFC:   data_out_final = FPGA_VER[31:24];
                        8'hFD:   data_out_final = FPGA_VER[23:16];
                        8'hFE:   data_out_final = FPGA_VER[15:8];
                        8'hFF:   data_out_final = FPGA_VER[7:0];
                        default: begin
                            if ((cmos_in_range == 1'b1) &&
                                (received_config == 1'b1)) begin
                                data_out_final = ram_out;   // NVRAM 内容
                            end
                            else begin
                                data_out_final = 8'h00;
                            end
                        end
                    endcase
                end
            end
            else begin
                // 既非读也非写
                bus_access    = 1'b0;
                bus_writer_en = 1'b0;
                data_out_final = 8'h00;
            end
        end
        else begin
            bus_access    = 1'b0;
            bus_writer_en = 1'b0;
        end

        DATA_OUT = data_out_final;
    end

    //=========================================================================
    // 1 Hz 节拍产生：1.1931818 MHz / 1193182
    //=========================================================================
    always @(posedge CLK_PIT) begin
        if (pit_divider >= PIT_DIV_1S) begin
            pit_divider <= 21'd0;
            tick_1hz    <= 1'b1;
        end
        else begin
            pit_divider <= pit_divider + 21'd1;
            tick_1hz    <= 1'b0;
        end
    end

    //=========================================================================
    // 闰年判断：年份能被 4 整除即可(年份 0-99 的简化逻辑)
    //=========================================================================
    assign is_leap_year = ((year % 8'd4) == 8'd0);

    //=========================================================================
    // 每月天数
    //=========================================================================
    always @(*) begin
        case (month)
            8'd4, 8'd6, 8'd9, 8'd11: max_days_in_month = 6'd30;   // 4/6/9/11 月
            8'd2:                    max_days_in_month = is_leap_year ? 6'd29 : 6'd28;
            default:                 max_days_in_month = 6'd31;
        endcase
    end

    //=========================================================================
    // 走时逻辑：秒 -> 分 -> 时 -> 星期/日期 -> 月 -> 年 -> 世纪
    //=========================================================================
    always @(posedge CLK_PIT) begin
        if (tick_1hz == 1'b1) begin
            if (seconds >= 8'd59) begin
                seconds <= 8'd0;

                if (minutes >= 8'd59) begin
                    minutes <= 8'd0;

                    if (hours >= 8'd23) begin
                        hours <= 8'd0;

                        // 星期 1..7 循环
                        if (weekday >= 8'd7) begin
                            weekday <= 8'd1;
                        end
                        else begin
                            weekday <= weekday + 8'd1;
                        end

                        // 日 / 月 / 年
                        if (day >= max_days_in_month) begin
                            day <= 8'd1;

                            if (month >= 8'd12) begin
                                month <= 8'd1;

                                if (year >= 8'd99) begin
                                    year    <= 8'd0;
                                    century <= century + 8'd1;
                                end
                                else begin
                                    year <= year + 8'd1;
                                end
                            end
                            else begin
                                month <= month + 8'd1;
                            end
                        end
                        else begin
                            day <= day + 8'd1;
                        end
                    end
                    else begin
                        hours <= hours + 8'd1;
                    end
                end
                else begin
                    minutes <= minutes + 8'd1;
                end
            end
            else begin
                seconds <= seconds + 8'd1;
            end
        end
    end

endmodule
