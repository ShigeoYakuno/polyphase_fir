// ============================================================================
// ファイル名 : polyphase_fir_stage1.sv  （逐次1DSP 版）
// 概要       : M=9 デシメーション用 FIR（63tap）
// 方式       : 9サンプル到着ごと（base_trigger）に、63tap を1DSPで逐次MAC
//              入力は止めない（10kHz）、システムクロック100MHzで余裕動作
// メモリ     : 単一の循環バッファ（64語, simple dual-port 推論）
// リソース   : DSP=1個、pf_buf/並列MAC/加算木は無し
// ============================================================================

module poly_fir_stage1 #(
    parameter int DECIMATION_FACTOR = 9,
    parameter int TAP_LEN          = 63,   // tap数
    parameter int DATA_WIDTH       = 16,
    parameter int COEF_WIDTH       = 16,
    parameter int BUF_LEN          = 64    
    )(
    input  logic                          clk,
    input  logic                          rst,
    input  logic                          din_valid,
    output logic                          dout_valid,
    input  logic signed [DATA_WIDTH-1:0]  din,
    output logic signed [DATA_WIDTH-1:0]  dout
    );

    // ------------------------------
    // 係数（63 taps）
    // 右シフト15bitでゲイン正規化（係数合 ? 2^15）
    // ------------------------------
    localparam logic signed [COEF_WIDTH-1:0] fir_coef [0:TAP_LEN-1] = '{
        16'd2,  16'd4,  16'd8,  16'd13,  16'd21,  16'd31,  16'd44,  16'd61,  16'd82,  16'd108, 
        16'd139,  16'd176,  16'd218,  16'd266,  16'd320,  16'd379,  16'd444,  16'd513,  16'd585,  16'd661, 
        16'd738,  16'd815,  16'd891,  16'd964,  16'd1033,  16'd1096,  16'd1153,  16'd1201,  16'd1239,  16'd1268, 
        16'd1285,  16'd1291,  16'd1285,  16'd1268,  16'd1239,  16'd1201,  16'd1153,  16'd1096,  16'd1033,  16'd964, 
        16'd891,  16'd815,  16'd738,  16'd661,  16'd585,  16'd513,  16'd444,  16'd379,  16'd320,  16'd266, 
        16'd218,  16'd176,  16'd139,  16'd108,  16'd82,  16'd61,  16'd44,  16'd31,  16'd21,  16'd13, 
        16'd8,  16'd4,  16'd2
    };

    // ------------------------------
    // ビット幅計算・定数
    // ------------------------------
    localparam int AW = $clog2(BUF_LEN);
    localparam int acc_width = (DATA_WIDTH + COEF_WIDTH) + $clog2(TAP_LEN);
    localparam signed [DATA_WIDTH-1:0] MAX_VAL = (1 <<< (DATA_WIDTH-1)) - 1;
    localparam signed [DATA_WIDTH-1:0] MIN_VAL = -(1 <<< (DATA_WIDTH-1));


    // ------------------------------
    // 入力用：循環バッファ（simple dual-port 推論）
    //  - Port A: 書込み（din_valid のとき1語）
    //  - Port B: 読み出し（逐次MAC中のみ）
    // ------------------------------
    (* ramstyle = "M9K, no_rw_check" *)
    logic signed [DATA_WIDTH-1:0] sram [0:BUF_LEN-1];

    logic [AW-1:0] wr_ptr;  // 次に書く位置
    always_ff @(posedge clk) begin : WR_PORT
        if (rst) begin
            wr_ptr <= '0;
        end else if (din_valid) begin
            sram[wr_ptr] <= din;
            wr_ptr <= (wr_ptr == BUF_LEN-1) ? '0 : (wr_ptr + 1);
        end
    end

    // 9サンプルカウンタ（出力計算のベーストリガ）
    logic [$clog2(DECIMATION_FACTOR)-1:0] phase_cnt;
    always_ff @(posedge clk) begin
        if (rst) phase_cnt <= '0;
        else if (din_valid) phase_cnt <= (phase_cnt == DECIMATION_FACTOR-1) ? '0 : (phase_cnt + 1);
    end
    wire base_trigger = (din_valid && (phase_cnt == DECIMATION_FACTOR-1));

    // ------------------------------
    // 逐次MAC用 FSM（1DSP）
    // ------------------------------
    typedef enum logic [1:0] {IDLE, BUSY, DONE} st_t;
    st_t st;

    logic [AW-1:0] rd_addr;
    logic signed [DATA_WIDTH-1:0] sample_q;
    always_ff @(posedge clk) begin
        sample_q <= sram[rd_addr]; // <- 無条件読出し
    end

    // スナップショット＆MAC管理
    logic [6:0]    tap_idx;           // 0..62
    logic [6:0]    tap_done;          // MAC済み数
    logic          have_prev;         // 前サイクル発行分のデータが有効
    logic signed [acc_width-1:0] acc_total;

    // 出力レジスタ
    logic signed [DATA_WIDTH-1:0] dout_reg;
    assign dout = dout_reg;

    // メインFSM
    always_ff @(posedge clk) begin
        if (rst) begin
            st          <= IDLE;
            dout_valid  <= 1'b0;
            dout_reg    <= '0;
            rd_addr     <= '0;
            tap_idx     <= '0;
            tap_done    <= '0;
            have_prev   <= 1'b0;
            acc_total   <= '0;
        end else begin
            dout_valid <= 1'b0; // 既定でLOW（1パルス出力）

            unique case (st)
                // --------------------------
                IDLE: begin
                    if (base_trigger) begin
                        // 直近に書いた位置
                        rd_addr   <= wr_ptr;
                        tap_idx   <= 7'd0;   // 係数 idx = 0 から
                        tap_done  <= 7'd0;
                        have_prev <= 1'b0;   // 次サイクルから sample_q が有効
                        acc_total <= '0;
                        st <= BUSY;
                    end
                end

                // --------------------------
                BUSY: begin
                    // 前サイクル発行分サンプルで MAC（have_prev=1 のとき）
                    if (have_prev) begin
                        acc_total <= acc_total+(sample_q*(* multstyle = "dsp" *)fir_coef[tap_idx]);
                        tap_idx  <= tap_idx + 1;
                        tap_done <= tap_done + 1;
                    end
                    // 次のアドレス（連続して1刻みで遡る）
                    rd_addr   <= (rd_addr == 0) ? (BUF_LEN-1) : (rd_addr - 1);
                    have_prev <= 1'b1;
                    // 63タップ完了判定
                    if (have_prev && (tap_done == TAP_LEN-1)) begin
                        // このサイクルで最後のMACを実行済み
                        st       <= DONE;
                        have_prev<= 1'b0; // 念のため下げる
                    end
                end

                // --------------------------
                DONE: begin
                    // スケーリング（>>>15）＆サチュレーション
                    logic signed [acc_width-1:0] scaled;
                    logic signed [DATA_WIDTH-1:0] saturated;

                    scaled = acc_total >>> 15;
                    if      (scaled >  MAX_VAL) saturated = MAX_VAL;
                    else if (scaled <  MIN_VAL) saturated = MIN_VAL;
                    else                        saturated = scaled[DATA_WIDTH-1:0];

                    dout_reg   <= saturated;
                    dout_valid <= 1'b1;
                    st         <= IDLE;
                end
                default: st <= IDLE;
            endcase
        end
    end

endmodule
