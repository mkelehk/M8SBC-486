//=============================================================================
// 模块名 : clock_section
// 功能   : 主时钟生成，48 MHz 输入 -> CPU / FPGA 系统时钟
// 项目   : Hamster 1 chipset (M8SBC-486 REV 1.0)
// 原文件 : clock_section.vhd
//
// 【本模块在系统中的位置】
//   顶层 m8sbc_main 的 CLKGEN 实例。输入是板上 48 MHz 有源晶振（GCK 引脚），
//   输出 CLK_OUT_CPU 直接接 486 CPU 的 CLK 脚，同时也是片内 CPU 侧逻辑的主时钟。
//
// 【当前版本：与 VHDL 逐拍等价的 fabric 分频】
//   本文件现在**逐句等价于** chipset_vhdl/clock_section.vhd：
//     IBUFG -> CLKDLL(只做 deskew，CLKDV_DIVIDE 固定 4.0，RST 接 0)
//           -> CLK0 -> BUFG_FB  -> CLKFB        （反馈，保证 CLK0 与 CLKIN 对齐）
//           -> CLK0 -> BUFG_SYS -> CLK_SYS(48MHz)
//           -> process(CLK_SYS)：LOCKED 有效时才按 DIVIDE_BY 分频
//           -> cpu_clk（寄存器输出）-> CPU_CLK_OUT
//
//   DIVIDE_BY = 2 -> 每个 CLK_SYS 周期翻转一次（24 MHz）
//   DIVIDE_BY = 3 -> cnt 计 0,1,2 后翻转（16 MHz）
//   DIVIDE_BY = 4 -> cnt 计 0,1,2,3 后翻转（12 MHz）
//   其它值        -> 直接把 CLK_SYS 打一拍输出（VHDL 的 fallback，原样保留）
//
//   LOCKED 无效时 cpu_clk 强制为 0，与 VHDL 的 "Optionally hold low until lock" 一致。
//
// 【为什么用 fabric 分频而不是 CLKDV 直接分频】（已上板验证的结论）
//   曾经试过 "CLKDLL 的 CLKDV 直接分频 + 自带 RST 上电计数器"，把
//   CLKDV_DIVIDE 绑到 DIVIDE_BY（默认 2.0）。上板后 CPU 完全不启动、
//   蜂鸣器长响。原因写在 VHDL 原作者这一行的注释里（clock_section.vhd 第 70 行）：
//       CLKDV_DIVIDE => 4.0,  -- used if DIVIDE_BY isn't 2 or 3. For some reason
//                             -- some values make the DLL not lock/wake (including 2 and 3)
//   即：本板 48 MHz 输入下 CLKDV_DIVIDE = 2.0 / 3.0 会让 DLL 锁不住。
//   DLL 不锁定 + STARTUP_WAIT="TRUE" -> GSR 一直被压住 -> 整片逻辑停在复位。
//   所以本文件与 VHDL 保持一致，用 CLK0 驱动 fabric 分频器，不再尝试 CLKDV。
//=============================================================================
`timescale 1ns / 1ps

module clock_section #(
    parameter integer DIVIDE_BY = 2      // 2 / 3 / 4，需与顶层等待周期常量配套修改
) (
    input  wire CLK_INPUT,               // 48 MHz 输入 (GCK 引脚)
    output wire CPU_CLK_OUT              // 分频后的 CPU / FPGA 时钟
);

    //-------------------------------------------------------------------------
    // 内部信号
    //-------------------------------------------------------------------------
    wire CLKIN_buf;      // IBUFG 之后的 48 MHz
    wire CLK0_raw;       // DLL 同相输出
    wire CLKFB;          // DLL 反馈输入（必须来自 BUFG）
    wire LOCKED_int;     // DLL 锁定标志：未锁定前分频器不工作
    wire CLK_SYS;        // 经 BUFG 的 48 MHz 系统时钟（分频器的时钟源）

    reg  cpu_clk = 1'b0; // 分频后的 CPU 时钟，初值 0（对应 VHDL 的 := '0'）

    // 2 位就够 /2 /3 /4 用（与 VHDL 的 unsigned(1 downto 0) 一致）
    reg  [1:0] cnt = 2'd0;

    //-------------------------------------------------------------------------
    // 48 MHz 源输入缓冲
    //-------------------------------------------------------------------------
    IBUFG CLKIN_IBUFG (
        .I(CLK_INPUT),
        .O(CLKIN_buf)
    );

    //-------------------------------------------------------------------------
    // DLL：只用来消除 48 MHz 的分布偏斜，不使用 CLKDV
    //
    // 三个参数的选择理由（与 VHDL 完全一致，不要改）：
    //   CLKDV_DIVIDE = 4.0  —— 该输出本身不用（CLKDV 悬空），写 4.0 只是沿用
    //                          VHDL 的值。原作者实测 2.0 / 3.0 会让 DLL 锁不住。
    //   DUTY_CYCLE_CORRECTION = TRUE —— 占空比 50% 校正。
    //   STARTUP_WAIT = TRUE —— GSR 保持到 DLL 锁定之后再释放，保证片内逻辑
    //                          不会在时钟还没稳定时就开始跑。
    //   RST = 1'b0 —— 直接接地（XAPP174 明确允许 "tied to ground"），
    //                 不使用上电复位脉冲；锁定状态由 LOCKED 反馈给分频器。
    //-------------------------------------------------------------------------
    CLKDLL #(
        .CLKDV_DIVIDE         (4.0),
        .DUTY_CYCLE_CORRECTION("TRUE"),
        .STARTUP_WAIT         ("TRUE")
    ) CLKDLL_inst (
        .CLKIN  (CLKIN_buf),
        .CLKFB  (CLKFB),
        .RST    (1'b0),
        .CLKDV  (),          // 不使用（对应 VHDL 的 open）
        .CLK0   (CLK0_raw),
        .CLK90  (),
        .CLK180 (),
        .CLK270 (),
        .CLK2X  (),
        .LOCKED (LOCKED_int)
    );

    //-------------------------------------------------------------------------
    // DLL 反馈缓冲：CLK0 经 BUFG 回到 CLKFB，DLL 靠它把 CLK0 与 CLKIN 相位对齐
    //-------------------------------------------------------------------------
    BUFG BUFG_FB (
        .I(CLK0_raw),
        .O(CLKFB)
    );

    //-------------------------------------------------------------------------
    // 系统时钟缓冲：48 MHz 进全局时钟网络，供分频器使用
    //-------------------------------------------------------------------------
    BUFG BUFG_SYS (
        .I(CLK0_raw),
        .O(CLK_SYS)
    );

    //-------------------------------------------------------------------------
    // 同步分频器（逐句对应 VHDL 的 process(CLK_SYS)）
    //
    //   LOCKED 无效 -> cpu_clk 保持 0，分频器不动作（且 cnt 不递增）
    //   DIVIDE_BY=2 -> 每拍翻转
    //   DIVIDE_BY=3 -> cnt 计到 2 时清零并翻转，周期 3 拍
    //   DIVIDE_BY=4 -> cnt 计到 3 时清零并翻转，周期 4 拍
    //   其它       -> 直接把 CLK_SYS 打一拍送出去
    //
    // 注意：这里用阻塞无关的普通非阻塞赋值，cpu_clk 是真实触发器，
    //       输出直接来自触发器 Q，保证与 VHDL 综合出的结构相同。
    //-------------------------------------------------------------------------
    always @(posedge CLK_SYS) begin
        if (LOCKED_int == 1'b1) begin
            if (DIVIDE_BY == 2) begin
                cpu_clk <= ~cpu_clk;             // 48/2 = 24 MHz
            end
            else if (DIVIDE_BY == 3) begin
                if (cnt == 2'd2) begin
                    cnt     <= 2'd0;
                    cpu_clk <= ~cpu_clk;         // 48/3 = 16 MHz
                end
                else begin
                    cnt <= cnt + 2'd1;
                end
            end
            else if (DIVIDE_BY == 4) begin
                if (cnt == 2'd3) begin
                    cnt     <= 2'd0;
                    cpu_clk <= ~cpu_clk;         // 48/4 = 12 MHz
                end
                else begin
                    cnt <= cnt + 2'd1;
                end
            end
            else begin
                cpu_clk <= CLK_SYS;              // fallback：不分频
            end
        end
        else begin
            cpu_clk <= 1'b0;                     // 未锁定：输出保持低
        end
    end

    assign CPU_CLK_OUT = cpu_clk;

endmodule
