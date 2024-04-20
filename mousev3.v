module top(
        input clk,
        input rst,
        input redin,
        input bluein,
        input greenin,
        output [3:0] vgaRed,
        output [3:0] vgaGreen,
        output [3:0] vgaBlue,
        output hsync,
        output vsync,
        output [3:0]AN,
        output [6:0]SEG,
        output reg flag_out,
        inout PS2_CLK,
        inout PS2_DATA
    );

    wire clk_25MHz;
    wire clk_segment;
    wire valid, flag;
    reg isX, isX_next, flag_rst = 1'b0;
    wire [9:0] h_cnt; //640
    wire [9:0] v_cnt;  //480
    wire [11:0] data; 
    //reg [11:0] data_next;
    wire [16:0] pixel_addr;
    wire [11:0] pixel;
    
    wire enable_mouse_display;
    wire [9 : 0] MOUSE_X_POS , MOUSE_Y_POS;
    wire MOUSE_LEFT , MOUSE_MIDDLE , MOUSE_RIGHT , MOUSE_NEW_EVENT;
    wire [3 : 0] mouse_cursor_red , mouse_cursor_green , mouse_cursor_blue;
    
    wire [11:0] mouse_pixel = {mouse_cursor_red, mouse_cursor_green, mouse_cursor_blue};

    wire rst_db, rst_op;
    debounce DB5(.s(rst), .s_db(rst_db), .clk(clk));
	onepulse OP1(.s(rst_db), .s_op(rst_op), .clk(clk_segment));

    assign {vgaRed, vgaGreen, vgaBlue} = (valid==1'b1) ? ((enable_mouse_display) ? mouse_pixel : pixel) : 12'h0;
    //assign data = (flag_rst) ? 12'hFFF : data;
    //assign flag = (flag_rst) ? 1'b1 : flag;

    clock_divisor clk_wiz_0_inst(
        .clk(clk),
        .clk1(clk_25MHz),
        .clk17(clk_segment)
    );

    pixel_gen pixel_gen_inst(
        .clk(clk_25MHz),
        .valid(valid),
        .rst(flag_rst),
        .red(redin),
        .blue(bluein),
        .green(greenin),
        .enable_mouse_display(enable_mouse_display),
        .mouse_pixel(mouse_pixel),
        .MOUSE_LEFT(MOUSE_LEFT),
        .MOUSE_RIGHT(MOUSE_RIGHT),
        .xpoint(MOUSE_X_POS),
        .ypoint(MOUSE_Y_POS),
        .data(data),
        .flag(flag)
    );
    
    segment_display s1(
        .clk(clk_segment),
        .MOUSE_X_POS(MOUSE_X_POS),
        .MOUSE_Y_POS(MOUSE_Y_POS),
        .isX(isX),
        .AN(AN),
        .SEG(SEG)
    );

    mem_addr_gen mem_addr_gen_inst(
        .clk(clk_segment),
        .rst(rst_op),
        .h_cnt(h_cnt),
        .v_cnt(v_cnt),
        .pixel_addr(pixel_addr)
    );
     
 
    blk_mem_gen_0 blk_mem_gen_0_inst(
        .clka(clk_25MHz),
        .wea(flag),
        .addra(pixel_addr),
        .dina(data),
        .douta(pixel),
        .rsta(0)
    ); 

    vga_controller vga_inst(
        .pclk(clk_25MHz),
        .reset(rst_op),
        .hsync(hsync),
        .vsync(vsync),
        .valid(valid),
        .h_cnt(h_cnt),
        .v_cnt(v_cnt)
    );
    
    mouse mouse_ctrl_inst(
        .clk(clk),
        .h_cntr_reg(h_cnt),
        .v_cntr_reg(v_cnt),
        .enable_mouse_display(enable_mouse_display),
        .MOUSE_X_POS(MOUSE_X_POS),
        .MOUSE_Y_POS(MOUSE_Y_POS),
        .MOUSE_LEFT(MOUSE_LEFT),
        .MOUSE_MIDDLE(MOUSE_MIDDLE),
        .MOUSE_RIGHT(MOUSE_RIGHT),
        .MOUSE_NEW_EVENT(MOUSE_NEW_EVENT),
        .mouse_cursor_red(mouse_cursor_red),
        .mouse_cursor_green(mouse_cursor_green),
        .mouse_cursor_blue(mouse_cursor_blue),
        .PS2_CLK(PS2_CLK),
        .PS2_DATA(PS2_DATA)
    );
    
    always@(posedge clk)begin
        if(rst) begin
            isX <= 1'b1;
        end else begin
            flag_out <= flag_rst;
            isX <= isX_next;
        end
    end
    
    always@(*) begin
        isX_next = isX;
        //data_next = (flag_rst) ? 12'hFFF : data;
        //flag_next = (flag_rst) ? 1'b1 : flag;
        if(rst && !flag_rst) flag_rst = 1'b1;
        //if(pixel_addr > 17'd76800) flag_rst = 1'b0;
        if(MOUSE_LEFT) begin
            isX_next = 1'b1;
        end else if(MOUSE_RIGHT) begin
            isX_next = 1'b0;
        end
    end
      
endmodule

module clock_divisor(clk17, clk1, clk);
    input clk;
    output clk1;
    output clk17;

    reg [17:0] num;
    wire [17:0] next_num;

    always @(posedge clk) begin
    num <= next_num;
    end

    assign next_num = num + 1'b1;
    assign clk1 = num[1];
    assign clk17 = num[17];
endmodule

module segment_display(
    input clk,
    input [9:0] MOUSE_X_POS,
    input [9:0] MOUSE_Y_POS,
    input isX,
    output reg [3:0] AN,
    output reg [6:0] SEG
    );

    reg [3:0] AN_next;
    wire [9:0] display_value;
    wire [3:0] DIGIT0, DIGIT1, DIGIT2;
    wire [3:0] value;

    assign display_value = (isX) ? MOUSE_X_POS: MOUSE_Y_POS;
    assign DIGIT0 = display_value % 10;
    assign DIGIT1 = ( display_value / 10 ) % 10;
    assign DIGIT2 = ( display_value / 100 ) % 10;

    always@(posedge clk)begin
    AN <= AN_next;
    end

    always@(*)begin
    AN_next = 4'b1110;
    if(AN==4'b1110) begin
        AN_next = 4'b1101;
    end else if(AN==4'b1101) begin
        AN_next = 4'b1011;
    end else if(AN==4'b1011) begin
        AN_next = 4'b1110;
    end
    end

    assign value = (AN==4'b1110) ? DIGIT0:
                (AN==4'b1101) ? DIGIT1:
                (AN==4'b1011) ? DIGIT2:
                                4'b1111;

    always@(*)begin
    case(value)
        4'd0: SEG = 7'b0000001;
        4'd1: SEG = 7'b1001111;
        4'd2: SEG = 7'b0010010;
        4'd3: SEG = 7'b0000110;
        4'd4: SEG = 7'b1001100;
        4'd5: SEG = 7'b0100100;
        4'd6: SEG = 7'b0100000;
        4'd7: SEG = 7'b0001111;
        4'd8: SEG = 7'b0000000;
        4'd9: SEG = 7'b0000100;
        default: SEG = 7'b1111111;
    endcase
    end
endmodule

module pixel_gen(
    input clk,
    input valid,
    input rst,
    input red,
    input blue,
    input green,
    input enable_mouse_display,
    input [11:0] mouse_pixel,
    input MOUSE_LEFT,
    input MOUSE_RIGHT,
    input xpoint,
    input ypoint,
    output reg [11:0] data,
    output reg flag
    );

    always @(*) begin
        if(rst)begin
            data<= 12'hFFF;
            flag<= 1'b1;
        end
        else if(!valid) begin
            data<= 12'hFFF;
            flag<= 1'b1;
        end
        else if(enable_mouse_display) begin
            if(MOUSE_LEFT) begin
                if(red && blue && green) data <= 12'hFFF;
                else if(red && !blue && !green) data <= 12'hF00;
                else if(!red && blue && !green) data <= 12'h0F0;
                else if(!red && !blue && green) data <= 12'h00F;
                else if(red && blue && !green) data <= 12'hFF0;
                else if(!red && blue && green) data <= 12'h0FF;
                else if(red && !blue && green) data <= 12'hF0F;
                else if(!red && !blue && !green) data <= 12'h000;
                flag <= 1'b1;
            end
            else if(MOUSE_RIGHT)begin
                data <= 12'hFFF;
                flag <= 1'b1;
            end
        end
        else begin
            data <= 12'hFFF;
            flag <= 1'b0;
        end
	end
endmodule

module mem_addr_gen(
   input clk,
   input rst,
   input [9:0] h_cnt,
   input [9:0] v_cnt,
   output [16:0] pixel_addr
   );
    
   //reg [7:0] position;
  
   //assign pixel_addr = ((h_cnt>>1)+320*(v_cnt>>1)+ position*320 )% 76800;  //640*480 --> 320*240 
   assign pixel_addr = ((h_cnt>>1)+320*(v_cnt>>1))% 76800;  //640*480 --> 320*240 

    /*
   always @ (posedge clk or posedge rst) begin
      if(rst)
          position <= 0;
       else if(position < 239)
           position <= position + 1;
       else
           position <= 0;
   end
   */
endmodule

module vga_controller(pclk, reset, hsync, vsync, valid, h_cnt, v_cnt);
    input pclk, reset;
    output hsync, vsync;
	output valid;
    output [9:0]h_cnt, v_cnt;
    
    reg [9:0]pixel_cnt;
    reg [9:0]line_cnt;
    reg hsync_i,vsync_i;
	wire hsync_default, vsync_default;
    wire [9:0] HD, HF, HS, HB, HT, VD, VF, VS, VB, VT;
	
    assign HD = 640;
    assign HF = 16;
    assign HS = 96;
    assign HB = 48;
    assign HT = 800; 
    assign VD = 480;
    assign VF = 10;
    assign VS = 2;
    assign VB = 33;
    assign VT = 525;
    assign hsync_default = 1'b1;
    assign vsync_default = 1'b1;
     
    always@(posedge pclk)
        if(reset)
            pixel_cnt <= 0;
        else if(pixel_cnt < (HT - 1))
                pixel_cnt <= pixel_cnt + 1;
             else
                pixel_cnt <= 0;

    always@(posedge pclk)
        if(reset)
            hsync_i <= hsync_default;
        else if((pixel_cnt >= (HD + HF - 1))&&(pixel_cnt < (HD + HF + HS - 1)))
                hsync_i <= ~hsync_default;
            else
                hsync_i <= hsync_default; 
    
    always@(posedge pclk)
        if(reset)
            line_cnt <= 0;
        else if(pixel_cnt == (HT -1))
                if(line_cnt < (VT - 1))
                    line_cnt <= line_cnt + 1;
                else
                    line_cnt <= 0;
                  
				  
    always@(posedge pclk)
        if(reset)
            vsync_i <= vsync_default; 
        else if((line_cnt >= (VD + VF - 1))&&(line_cnt < (VD + VF + VS - 1)))
            vsync_i <= ~vsync_default;
        else
            vsync_i <= vsync_default; 

    assign hsync = hsync_i;
    assign vsync = vsync_i;
    assign valid = ((pixel_cnt < HD) && (line_cnt < VD));
	
    assign h_cnt = (pixel_cnt < HD)? pixel_cnt : 10'd0;//639
    assign v_cnt = (line_cnt < VD)? line_cnt : 10'd0;//479
endmodule

module mouse(
        input clk,
        input [9 : 0] h_cntr_reg,
        input [9 : 0] v_cntr_reg,
        output enable_mouse_display,
        output [9 : 0] MOUSE_X_POS,
        output [9 : 0] MOUSE_Y_POS,
        output MOUSE_LEFT,
        output MOUSE_MIDDLE,
        output MOUSE_RIGHT,
        output MOUSE_NEW_EVENT,
        output [3 : 0] mouse_cursor_red,
        output [3 : 0] mouse_cursor_green,
        output [3 : 0] mouse_cursor_blue,
        inout PS2_CLK,
        inout PS2_DATA
    );

    wire [3:0] MOUSE_Z_POS;
    
    MouseCtl #(
      .SYSCLK_FREQUENCY_HZ(108000000),
      .CHECK_PERIOD_MS(500),
      .TIMEOUT_PERIOD_MS(100)
    )MC1(
        .clk(clk),
        .rst(1'b0),
        .xpos(MOUSE_X_POS),
        .ypos(MOUSE_Y_POS),
        .zpos(MOUSE_Z_POS),
        .left(MOUSE_LEFT),
        .middle(MOUSE_MIDDLE),
        .right(MOUSE_RIGHT),
        .new_event(MOUSE_NEW_EVENT),
        .value(12'd0),
        .setx(1'b0),
        .sety(1'b0),
        .setmax_x(1'b0),
        .setmax_y(1'b0),
        .ps2_clk(PS2_CLK),
        .ps2_data(PS2_DATA)
    );
    
    MouseDisplay MD1(
        .pixel_clk(clk),
        .xpos(MOUSE_X_POS),
        .ypos(MOUSE_Y_POS),
        .hcount(h_cntr_reg),
        .vcount(v_cntr_reg),
        .enable_mouse_display_out(enable_mouse_display),
        .red_out(mouse_cursor_red),
        .green_out(mouse_cursor_green),
        .blue_out(mouse_cursor_blue)
    );
endmodule

module onepulse(s, s_op, clk);
	input s, clk;
	output reg s_op;
	reg s_delay;
	always@(posedge clk)begin
		s_op <= s&(!s_delay);
		s_delay <= s;
	end
endmodule

module debounce(s, s_db, clk);
	input s, clk;
	output s_db;
	reg [3:0] DFF;
	
	always@(posedge clk)begin
		DFF[3:1] <= DFF[2:0];
		DFF[0] <= s;
	end
	assign s_db = (DFF == 4'b1111)? 1'b1 : 1'b0;
endmodule