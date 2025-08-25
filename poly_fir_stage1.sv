// ============================================================================
// �t�@�C���� : polyphase_fir_stage1.sv  �i����1DSP �Łj
// �T�v       : M=9 �f�V���[�V�����p FIR�i63tap�j
// ����       : 9�T���v���������Ɓibase_trigger�j�ɁA63tap ��1DSP�Œ���MAC
//              ���͎͂~�߂Ȃ��i10kHz�j�A�V�X�e���N���b�N100MHz�ŗ]�T����
// ������     : �P��̏z�o�b�t�@�i64��, simple dual-port ���_�j
// ���\�[�X   : DSP=1�Apf_buf/����MAC/���Z�؂͖���
// ============================================================================

module poly_fir_stage1 #(
    parameter int DECIMATION_FACTOR = 9,
    parameter int TAP_LEN          = 63,   // tap��
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
    // �W���i63 taps�j
    // �E�V�t�g15bit�ŃQ�C�����K���i�W���� ? 2^15�j
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
    logic [6:0]    tap_idx;           // 0..62
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
                        acc_total <= acc_total+(sample_q*(* multstyle = "dsp" *)fir_coef[tap_idx]);
                        tap_idx  <= tap_idx + 1;
                        tap_done <= tap_done + 1;
                    end
                    // ���̃A�h���X�i�A������1���݂ők��j
                    rd_addr   <= (rd_addr == 0) ? (BUF_LEN-1) : (rd_addr - 1);
                    have_prev <= 1'b1;
                    // 63�^�b�v��������
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
