
module poly_fir_stage3 #(
    parameter int DECIMATION_FACTOR = 7,
    parameter int TAP_LEN          = 49,   // tap��
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


    localparam logic signed [COEF_WIDTH-1:0] fir_coef [0:TAP_LEN-1] = '{
        16'd4,  16'd9,  16'd17,  16'd30,  16'd49,  16'd75,  16'd110,  16'd154,  16'd209,  16'd276, 
        16'd354,  16'd443,  16'd544,  16'd653,  16'd770,  16'd891,  16'd1014,  16'd1135,  16'd1249,  16'd1354, 
        16'd1446,  16'd1521,  16'd1577,  16'd1612,  16'd1623,  16'd1612,  16'd1577,  16'd1521,  16'd1446,  16'd1354, 
        16'd1249,  16'd1135,  16'd1014,  16'd891,  16'd770,  16'd653,  16'd544,  16'd443,  16'd354,  16'd276, 
        16'd209,  16'd154,  16'd110,  16'd75,  16'd49,  16'd30,  16'd17,  16'd9,  16'd4
    };

    // ------------------------------
    // �r�b�g���v�Z�E�萔
    // ------------------------------
    localparam int AW = $clog2(BUF_LEN);
    localparam int acc_width = (DATA_WIDTH + COEF_WIDTH) + $clog2(TAP_LEN);
    localparam signed [DATA_WIDTH-1:0] MAX_VAL = (1 <<< (DATA_WIDTH-1)) - 1;
    localparam signed [DATA_WIDTH-1:0] MIN_VAL = -(1 <<< (DATA_WIDTH-1));


    // ------------------------------
    // ���͗p�F�z�o�b�t�@�isimple dual-port ���_�j
    //  - Port A: �����݁idin_valid �̂Ƃ�1��j
    //  - Port B: �ǂݏo���i����MAC���̂݁j
    // ------------------------------
    (* ramstyle = "M9K, no_rw_check" *)
    logic signed [DATA_WIDTH-1:0] sram [0:BUF_LEN-1];

    logic [AW-1:0] wr_ptr;  // ���ɏ����ʒu
    always_ff @(posedge clk) begin : WR_PORT
        if (rst) begin
            wr_ptr <= '0;
        end else if (din_valid) begin
            sram[wr_ptr] <= din;
            wr_ptr <= (wr_ptr == BUF_LEN-1) ? '0 : (wr_ptr + 1);
        end
    end

    // 9�T���v���J�E���^�i�o�͌v�Z�̃x�[�X�g���K�j
    logic [$clog2(DECIMATION_FACTOR)-1:0] phase_cnt;
    always_ff @(posedge clk) begin
        if (rst) phase_cnt <= '0;
        else if (din_valid) phase_cnt <= (phase_cnt == DECIMATION_FACTOR-1) ? '0 : (phase_cnt + 1);
    end
    wire base_trigger = (din_valid && (phase_cnt == DECIMATION_FACTOR-1));

    // ------------------------------
    // ����MAC�p FSM�i1DSP�j
    // ------------------------------
    typedef enum logic [1:0] {IDLE, BUSY, DONE} st_t;
    st_t st;

    logic [AW-1:0] rd_addr;
    logic signed [DATA_WIDTH-1:0] sample_q;
    always_ff @(posedge clk) begin
        sample_q <= sram[rd_addr]; // <- �������Ǐo��
    end

    // �X�i�b�v�V���b�g��MAC�Ǘ�
    logic [6:0]    tap_idx;           
    logic [6:0]    tap_done;          // MAC�ςݐ�
    logic          have_prev;         // �O�T�C�N�����s���̃f�[�^���L��
    logic signed [acc_width-1:0] acc_total;

    // �o�̓��W�X�^
    logic signed [DATA_WIDTH-1:0] dout_reg;
    assign dout = dout_reg;

    // ���C��FSM
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
            dout_valid <= 1'b0; // �����LOW�i1�p���X�o�́j

            unique case (st)
                // --------------------------
                IDLE: begin
                    if (base_trigger) begin
                        // ���߂ɏ������ʒu
                        rd_addr   <= wr_ptr;
                        tap_idx   <= 7'd0;   // �W�� idx = 0 ����
                        tap_done  <= 7'd0;
                        have_prev <= 1'b0;   // ���T�C�N������ sample_q ���L��
                        acc_total <= '0;
                        st <= BUSY;
                    end
                end

                // --------------------------
                BUSY: begin
                    // �O�T�C�N�����s���T���v���� MAC�ihave_prev=1 �̂Ƃ��j
                    if (have_prev) begin
                        acc_total<=acc_total+(sample_q*(*multstyle="dsp"*)fir_coef[tap_idx]);
                        tap_idx  <= tap_idx + 1;
                        tap_done <= tap_done + 1;
                    end
                    // ���̃A�h���X�i�A������1���݂ők��j
                    rd_addr   <= (rd_addr == 0) ? (BUF_LEN-1) : (rd_addr - 1);
                    have_prev <= 1'b1;
                    // ��������
                    if (have_prev && (tap_done == TAP_LEN-1)) begin
                        // ���̃T�C�N���ōŌ��MAC�����s�ς�
                        st       <= DONE;
                        have_prev<= 1'b0; // �O�̂��߉�����
                    end
                end

                // --------------------------
                DONE: begin
                    // �X�P�[�����O�i>>>15�j���T�`�����[�V����
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
