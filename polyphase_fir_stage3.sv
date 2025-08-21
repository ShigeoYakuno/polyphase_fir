//==============================================================================
// Polyphase FIR Filter (49 Taps, 1/7 Decimation)
//==============================================================================
module polyphase_fir_stage3 #(
    parameter int DECIMATION_FACTOR = 7,
    parameter int TAP_LEN_ORIG      = 49,
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

    localparam TAP_LEN_PADDED = TAP_LEN_ORIG;
    localparam SUB_TAP_LEN = TAP_LEN_PADDED / DECIMATION_FACTOR;
    localparam int mul_width = DATA_WIDTH + COEF_WIDTH;
    localparam int sub_acc_width = mul_width + $clog2(SUB_TAP_LEN); // 7タップ分の積和
    localparam int final_acc_width = sub_acc_width + $clog2(DECIMATION_FACTOR); // 7個の和
    localparam signed [DATA_WIDTH-1:0] MAX_VAL = (1 << (DATA_WIDTH - 1)) - 1;
    localparam signed [DATA_WIDTH-1:0] MIN_VAL = -(1 << (DATA_WIDTH - 1));

    // ============================================================================
    // フィルタ係数 (49 taps)
    // ============================================================================
    localparam logic signed [COEF_WIDTH-1:0] fir_coef [TAP_LEN_ORIG-1:0] = '{
        16'd4,  16'd9,  16'd17,  16'd30,  16'd49,  16'd75,  16'd110,  16'd154,  16'd209,  16'd276, 
        16'd354,  16'd443,  16'd544,  16'd653,  16'd770,  16'd891,  16'd1014,  16'd1135,  16'd1249,  16'd1354, 
        16'd1446,  16'd1521,  16'd1577,  16'd1612,  16'd1623,  16'd1612,  16'd1577,  16'd1521,  16'd1446,  16'd1354, 
        16'd1249,  16'd1135,  16'd1014,  16'd891,  16'd770,  16'd653,  16'd544,  16'd443,  16'd354,  16'd276, 
        16'd209,  16'd154,  16'd110,  16'd75,  16'd49,  16'd30,  16'd17,  16'd9,  16'd4
    };

    typedef logic signed [COEF_WIDTH-1:0] T_SubFilterCoefs[SUB_TAP_LEN-1:0];
    typedef T_SubFilterCoefs T_PolyCoefs[DECIMATION_FACTOR-1:0];

    localparam T_PolyCoefs poly_coefs = get_poly_coefs();

    // 初期化用の関数を定義
    function automatic T_PolyCoefs get_poly_coefs();
        T_PolyCoefs temp_coefs;
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

    generate
        genvar k;
        for (k = 0; k < DECIMATION_FACTOR; k = k + 1) begin : sub_fir_mac

            logic signed [DATA_WIDTH-1:0] sr_bank_reg [SUB_TAP_LEN-1:0];
            logic signed [sub_acc_width-1:0] mac_pipe_acc;
            logic calc_trigger_pipe;
            logic calc_trigger_pipe2;

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
                    calc_trigger_pipe2 <= 1'b0;
                end else begin
                    calc_trigger_pipe <= calc_trigger;
                    calc_trigger_pipe2 <= calc_trigger_pipe;
                    if (calc_trigger_pipe) begin
                        automatic logic signed [sub_acc_width-1:0] sum = '0;
                        for (int tap = 0; tap < 4; tap++) begin
                            sum += sr_bank_reg[tap] * poly_coefs[k][tap];
                        end
                        mac_pipe_acc <= sum;
                    end
                end
            end
            // パイプラインステージ2: 残りの3タップを計算して最終結果を出力
            always_ff @(posedge clk) begin
                if (rst) begin
                    sub_filter_out[k] <= '0;
                end else if (calc_trigger_pipe2) begin
                    automatic logic signed [sub_acc_width-1:0] sum = mac_pipe_acc;
                    for (int tap = 4; tap < SUB_TAP_LEN; tap++) begin
                        sum += sr_bank_reg[tap] * poly_coefs[k][tap];
                    end
                    sub_filter_out[k] <= sum;
                end
            end
        end

    endgenerate

    // ============================================================================
    // パイプライン化された加算器ツリー (7入力 -> 3サイクル遅延)
    // ============================================================================
    logic signed [sub_acc_width+0:0] sum_stage1 [3:0]; // 7->4
    logic signed [sub_acc_width+1:0] sum_stage2 [1:0]; // 4->2
    logic signed [final_acc_width-1:0] total_sum;      // 2->1

    // Stage 1: 7 -> 4 （[0+1],[2+3],[4+5], [6]素通し）
    always_ff @(posedge clk) begin
        if (rst) begin
            sum_stage1 <= '{default:'0};
        end else begin
            sum_stage1[0] <= sub_filter_out[0] + sub_filter_out[1];
            sum_stage1[1] <= sub_filter_out[2] + sub_filter_out[3];
            sum_stage1[2] <= sub_filter_out[4] + sub_filter_out[5];
            sum_stage1[3] <= sub_filter_out[6];           // 残り1本は素通し
        end
    end

    // Stage 2: 4 -> 2
    always_ff @(posedge clk) begin
        if (rst) begin
            sum_stage2 <= '{default:'0};
        end else begin
            sum_stage2[0] <= sum_stage1[0] + sum_stage1[1];
            sum_stage2[1] <= sum_stage1[2] + sum_stage1[3];
        end
    end

    // Stage 3: 2 -> 1
    always_ff @(posedge clk) begin
        if (rst) begin
            total_sum <= '0;
        end else begin
            total_sum <= sum_stage2[0] + sum_stage2[1];
        end
    end


    // ============================================================================
    // 出力ステージ
    // ============================================================================
    logic [5:0] valid_pipeline;
    always_ff @(posedge clk) begin
        if (rst) valid_pipeline <= '0;
        else valid_pipeline <= {valid_pipeline[4:0], calc_trigger};
    end

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
            dout_valid <= valid_pipeline[5];

            if (valid_pipeline[5]) begin
                logic signed [final_acc_width-1:0] scaled_out;
                logic signed [DATA_WIDTH-1:0] saturated_out;
                // スケーリング
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