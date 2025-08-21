//==============================================================================
// Polyphase FIR Filter (63 Taps, 1/9 Decimation)
//==============================================================================
module polyphase_fir_stage1 #(
    parameter int DECIMATION_FACTOR = 9,
    parameter int TAP_LEN_ORIG      = 63,
    parameter int DATA_WIDTH        = 16,
    parameter int COEF_WIDTH        = 16
)(
    input  logic clk,
    input  logic rst,
    input  logic din_valid,
    output logic dout_valid,
    input  logic signed [DATA_WIDTH-1:0] din,
    output logic signed [DATA_WIDTH-1:0] dout
);

    // ============================================================================
    // パラメータ計算
    // ============================================================================
    // 63は9で割り切れるため、パディングは不要
    localparam TAP_LEN_PADDED = TAP_LEN_ORIG;
    // サブフィルタあたりのタップ長 -> 63 / 9 = 7
    localparam SUB_TAP_LEN = TAP_LEN_PADDED / DECIMATION_FACTOR;

    // 演算に必要なビット幅
    localparam int mul_width = DATA_WIDTH + COEF_WIDTH;
    localparam int sub_acc_width = mul_width + $clog2(SUB_TAP_LEN); // 7タップ分の積和
    localparam int final_acc_width = sub_acc_width + $clog2(DECIMATION_FACTOR); // 9個の和

    // オーバーフロー対策用の最大値・最小値
    localparam signed [DATA_WIDTH-1:0] MAX_VAL = (1 << (DATA_WIDTH - 1)) - 1;
    localparam signed [DATA_WIDTH-1:0] MIN_VAL = -(1 << (DATA_WIDTH - 1));

    // ============================================================================
    // フィルタ係数 (63 taps)
    // ============================================================================
    localparam logic signed [COEF_WIDTH-1:0] fir_coef [TAP_LEN_ORIG-1:0] = '{
        16'd2,  16'd4,  16'd8,  16'd13,  16'd21,  16'd31,  16'd44,  16'd61,  16'd82,  16'd108, 
        16'd139,  16'd176,  16'd218,  16'd266,  16'd320,  16'd379,  16'd444,  16'd513,  16'd585,  16'd661, 
        16'd738,  16'd815,  16'd891,  16'd964,  16'd1033,  16'd1096,  16'd1153,  16'd1201,  16'd1239,  16'd1268, 
        16'd1285,  16'd1291,  16'd1285,  16'd1268,  16'd1239,  16'd1201,  16'd1153,  16'd1096,  16'd1033,  16'd964, 
        16'd891,  16'd815,  16'd738,  16'd661,  16'd585,  16'd513,  16'd444,  16'd379,  16'd320,  16'd266, 
        16'd218,  16'd176,  16'd139,  16'd108,  16'd82,  16'd61,  16'd44,  16'd31,  16'd21,  16'd13, 
        16'd8,  16'd4,  16'd2
    };

    // ★★★ 修正1-1: typedefでアンパック配列の型を定義 ★★★
    typedef logic signed [COEF_WIDTH-1:0] T_SubFilterCoefs[SUB_TAP_LEN-1:0];
    typedef T_SubFilterCoefs T_PolyCoefs[DECIMATION_FACTOR-1:0];

    // ポリフェーズフィルタ用の2次元係数配列
    // ★★★ 修正1-2: 新しく定義した型を使用 ★★★
    localparam T_PolyCoefs poly_coefs = get_poly_coefs();

    // 初期化用の関数を定義
    // ★★★ 修正1-3: 関数の戻り値にも同じ型を使用 ★★★
    function automatic T_PolyCoefs get_poly_coefs();
        T_PolyCoefs temp_coefs; // ★★★ 型の利用 ★★★
        for (int i = 0; i < DECIMATION_FACTOR; i++) begin
            for (int j = 0; j < SUB_TAP_LEN; j++) begin
                temp_coefs[i][j] = fir_coef[j * DECIMATION_FACTOR + i];
            end
        end
        return temp_coefs;
    endfunction

    // ============================================================================
    // 入力ステージ
    // ============================================================================
    logic [$clog2(DECIMATION_FACTOR)-1:0] input_cnt;
    logic signed [DATA_WIDTH-1:0] sr_bank [DECIMATION_FACTOR-1:0][SUB_TAP_LEN-1:0];
    logic calc_trigger;

    always_ff @(posedge clk) begin
        if (rst) begin
            input_cnt <= '0;
            for (int i = 0; i < DECIMATION_FACTOR; i++) begin
                for (int j = 0; j < SUB_TAP_LEN; j++) begin
                    sr_bank[i][j] <= '0;
                end
            end
        end else if (din_valid) begin
            for (int j = SUB_TAP_LEN - 1; j > 0; j--) begin
                sr_bank[input_cnt][j] <= sr_bank[input_cnt][j-1];
            end
            sr_bank[input_cnt][0] <= din;

            if (input_cnt == DECIMATION_FACTOR - 1) begin
                input_cnt <= '0;
            end else begin
                input_cnt <= input_cnt + 1;
            end
        end
    end
    
    always_ff @(posedge clk) begin
        if (rst) calc_trigger <= 1'b0;
        else     calc_trigger <= (din_valid && input_cnt == DECIMATION_FACTOR - 1);
    end

 // ============================================================================
 // 並列フィルタ計算ステージ (1サイクル遅延)
 // ============================================================================
    logic signed [sub_acc_width-1:0] sub_filter_out [DECIMATION_FACTOR-1:0];

 // generateブロック内
    generate
        genvar k;
        for (k = 0; k < DECIMATION_FACTOR; k = k + 1) begin : sub_fir_mac

            // ★★★ 修正点1: 計算に使うデータを保持するレジスタを追加 ★★★
            logic signed [DATA_WIDTH-1:0] sr_bank_reg [SUB_TAP_LEN-1:0];
            
            // パイプライン用の中間レジスタを定義
            logic signed [sub_acc_width-1:0] mac_pipe_acc;
            logic calc_trigger_pipe;
            // ★★★ 修正点5-0: 追加レジスタ宣言 (ブロック内) ★★★
            logic calc_trigger_pipe2;

            // ★★★ 修正2: forループで要素ごとに代入する ★★★
            always_ff @(posedge clk) begin
                if (calc_trigger) begin
                    for (int i = 0; i < SUB_TAP_LEN; i = i + 1) begin
                        sr_bank_reg[i] <= sr_bank[k][i];
                    end
                end
            end

            // パイプラインステージ1: 最初の4タップを計算
            always_ff @(posedge clk) begin
                if (rst) begin
                    mac_pipe_acc <= '0;
                    calc_trigger_pipe <= 1'b0;
                    // ★★★ 修正点5-1: Stage2用のバリッドを追加しリセット ★★★
                    calc_trigger_pipe2 <= 1'b0;
                end else begin
                    // ★★★ 修正点5-2: バリッドを1段ずつ進める ★★★
                    calc_trigger_pipe <= calc_trigger;
                    calc_trigger_pipe2 <= calc_trigger_pipe;
                    // ★★★ 修正点5-3: Stage1のゲートを calc_trigger_pipe に変更 ★★★
                    if (calc_trigger_pipe) begin
                        automatic logic signed [sub_acc_width-1:0] sum = '0;
                        for (int tap = 0; tap < 4; tap++) begin
                            // ★★★ 修正点3: sr_bankの代わりにラッチしたsr_bank_regを使用 ★★★
                            sum += sr_bank_reg[tap] * poly_coefs[k][tap];
                        end
                        mac_pipe_acc <= sum;
                    end
                end
            end

            // ★★★ 修正点5-4: Stage2のゲートを calc_trigger_pipe2 に変更 ★★★
            // パイプラインステージ2: 残りの3タップを計算して最終結果を出力
            always_ff @(posedge clk) begin
                if (rst) begin
                    sub_filter_out[k] <= '0;
                end else if (calc_trigger_pipe2) begin
                    automatic logic signed [sub_acc_width-1:0] sum = mac_pipe_acc;
                    for (int tap = 4; tap < SUB_TAP_LEN; tap++) begin
                        // ★★★ 修正点4: こちらもsr_bank_regを使用 ★★★
                        sum += sr_bank_reg[tap] * poly_coefs[k][tap];
                    end
                    sub_filter_out[k] <= sum;
                end
            end
        end

    endgenerate

    // ============================================================================
    // パイプライン化された加算器ツリー (9入力 -> 4サイクル遅延)
    // ============================================================================
    logic signed [sub_acc_width+0:0] sum_stage1 [4:0];
    logic signed [sub_acc_width+1:0] sum_stage2 [2:0];
    logic signed [sub_acc_width+2:0] sum_stage3 [1:0];
    logic signed [final_acc_width-1:0] total_sum;

    // Stage 1: 9 -> 5
    always_ff @(posedge clk) begin
        if (rst) begin
            sum_stage1 <= '{default:'0};
        end else begin
            for (int i = 0; i < 4; i++) sum_stage1[i] <= sub_filter_out[2*i] + sub_filter_out[2*i+1];
            sum_stage1[4] <= sub_filter_out[8];
        end
    end
    // Stage 2: 5 -> 3
    always_ff @(posedge clk) begin
        if (rst) begin
            sum_stage2 <= '{default:'0};
        end else begin
            for (int i = 0; i < 2; i++) sum_stage2[i] <= sum_stage1[2*i] + sum_stage1[2*i+1];
            sum_stage2[2] <= sum_stage1[4];
        end
    end
    // Stage 3: 3 -> 2
    always_ff @(posedge clk) begin
        if (rst) begin
            sum_stage3 <= '{default:'0};
        end else begin
            sum_stage3[0] <= sum_stage2[0] + sum_stage2[1];
            sum_stage3[1] <= sum_stage2[2];
        end
    end
    // Stage 4: 2 -> 1
    always_ff @(posedge clk) begin
        if (rst) begin
            total_sum <= '0;
        end else begin
            total_sum <= sum_stage3[0] + sum_stage3[1];
        end
    end

    // ============================================================================
    // 出力ステージ
    // ============================================================================
    // パイプラインの総遅延: 2 (MAC) + 4 (Adder Tree) + 1 (Data/Valid Align) = 7サイクル
    logic [6:0] valid_pipeline;
    always_ff @(posedge clk) begin
        if (rst) valid_pipeline <= '0;
        else valid_pipeline <= {valid_pipeline[5:0], calc_trigger};
    end
    // ★★★ 修正点6: Data/Valid Align 用の1段レジスタを追加 (案A) ★★★
    logic signed [final_acc_width-1:0] total_sum_q;
    always_ff @(posedge clk) begin
        if (rst) total_sum_q <= '0;
        else     total_sum_q <= total_sum;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            dout <= '0;
            dout_valid <= 1'b0;
        end else begin
            dout_valid <= valid_pipeline[6]; // ★★★ 常に最新のvalidを出力
            if (valid_pipeline[6]) begin // ★★★ 最終段でチェック
                logic signed [final_acc_width-1:0] scaled_out;
                logic signed [DATA_WIDTH-1:0] saturated_out;
                // この時点のtotal_sumは1サイクル前に確定しているので、データは正しい
                // ★★★ 修正点6-1: total_sum → total_sum_q に変更 ★★★
                scaled_out = total_sum_q >>> 15;
                // サチュレーション
                if (scaled_out > MAX_VAL) saturated_out = MAX_VAL;
                else if (scaled_out < MIN_VAL) saturated_out = MIN_VAL;
                else saturated_out = scaled_out[DATA_WIDTH-1:0];
                dout <= saturated_out;
            end
        end
    end

endmodule