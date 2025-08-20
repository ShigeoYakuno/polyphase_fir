//==============================================================================
// テストベンチ for polyphase_fir_decimator_63tap
//
// 機能:
// 1. input.txt から整数データを1行ずつ読み込む
// 2. 読み込んだデータをFIRフィルタの入力 (din) として与える
// 3. FIRフィルタからの有効な出力 (dout) を output.txt に書き出す
//==============================================================================
`timescale 1ns/1ps

module tb_polyphase_fir;

    // DUT (Device Under Test) のパラメータ
    localparam int DATA_WIDTH = 16;

    // テストベンチ内部で使用する信号
    logic clk;
    logic rst;
    logic din_valid;
    logic dout_valid;
    logic signed [DATA_WIDTH-1:0] din;
    logic signed [DATA_WIDTH-1:0] dout;

    // DUT (検証対象のFIRフィルタ) のインスタンス化
    polyphase_fir_decimator_63tap #(
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .din_valid  (din_valid),
        .dout_valid (dout_valid),
        .din        (din),
        .dout       (dout)
    );

    // 1. クロック生成 (5ns周期、つまり100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // ファイルハンドルと読み込み用変数
    integer input_file;
    integer output_file;
    integer read_data;
    integer scan_status;

    // 2. メインのテストシーケンス
    initial begin
        // ファイルを開く
        input_file = $fopen("C:/Users/yakun/screen/sim/polyphase_fir/input.txt", "r");
        
        if (input_file == 0) begin
            $display("エラー: input.txt を開けませんでした。");
            $finish;
        end
        
        output_file = $fopen("C:/Users/yakun/screen/sim/polyphase_fir/output.txt", "w");
        if (output_file == 0) begin
            $display("エラー: output.txt を作成できませんでした。");
            $finish;
        end

        // リセットシーケンス
        $display("シミュレーションを開始します...");
        rst <= 1;
        din_valid <= 0;
        din <= 0;
        repeat(5) @(posedge clk);
        rst <= 0;
        @(posedge clk);
        $display("リセットを解除しました。");

        // input.txtを最後まで読み込み、DUTに入力する
        while (!$feof(input_file)) begin
            // ファイルから1行読み込む
            scan_status = $fscanf(input_file, "%d", read_data);
            
            // 読み込みが成功した場合のみデータを入力
            if (scan_status == 1) begin
                @(posedge clk);
                din <= read_data;
                din_valid <= 1; // データを1クロックサイクルだけ有効にする
                
                @(posedge clk);
                din_valid <= 0;
            end
        end
        $display("input.txt の読み込みが完了しました。");

        // パイプラインに残っているデータをすべて吐き出すまで待つ
        // DUTの遅延は6サイクルなので、余裕をもって20サイクル待つ
        repeat(20) @(posedge clk);

        // クリーンアップとシミュレーション終了
        $display("シミュレーションを終了します。");
        $fclose(input_file);
        $fclose(output_file);
        $finish;
    end

    // 3. DUTからの出力をファイルに書き込む
    // dout_validがアサートされた時に、その時のdoutの値を書き込む
    always @(posedge clk) begin
        if (rst) begin
            // リセット中はなにもしない
        end else if (dout_valid) begin
            $fdisplay(output_file, "%d", dout);
        end
    end

endmodule