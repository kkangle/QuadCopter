module QuadCopter_tb();
			
//// Interconnects to DUT/support defined as type wire /////
wire SS_n,SCLK,MOSI,MISO,INT;
wire SS_A2D_n,SCLK_A2D,MOSI_A2D,MISO_A2D;
wire RX,TX;
wire [7:0] rx_data;				// response from DUT
wire cmd_sent,rx_rdy;
wire frnt_ESC, back_ESC, left_ESC, rght_ESC;

////// Stimulus is declared as type reg ///////
reg clk, rst_n;
reg [7:0] cmd;		// command to Copter via wireless link
reg [15:0] data;				// data associated with command
reg snd_cmd;					// asserted to initiate sending of command (to your CommMaster)
reg clr_frm_snt_out;				// asserted to knock down resp_rdy

/////// declare any localparams here /////
localparam REQ_BATT = 8'h01;
localparam SET_PTCH = 8'h02;
localparam SET_ROLL = 8'h03;
localparam SET_YAW = 8'h04;
localparam SET_THRST = 8'h05;
localparam CALIBRATE = 8'h06;
localparam EMER_LAND = 8'h07;
localparam MTRS_OFF = 8'h08;
localparam POS_ACK = 8'hA5;

////////////////////////////////////////////////////////////////
// Instantiate Physical Model of Copter with Inertial sensor //
//////////////////////////////////////////////////////////////	
CycloneIV iQuad(.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.INT(INT),
                .frnt_ESC(frnt_ESC),.back_ESC(back_ESC),.left_ESC(left_ESC),
				.rght_ESC(rght_ESC));				  

///////////////////////////////////////////////////
// Instantiate Model of A2D for battery voltage //
/////////////////////////////////////////////////
ADC128S iA2D(.clk(clk),.rst_n(rst_n),.SS_n(SS_A2D_n),.SCLK(SCLK_A2D),
             .MISO(MISO_A2D),.MOSI(MOSI_A2D));			
	 
////// Instantiate DUT ////////
QuadCopter iDUT(.clk(clk),.RST_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),
                .INT(INT),.RX(RX),.TX(TX),.LED(),.FRNT(frnt_ESC),.BCK(back_ESC),
				.LFT(left_ESC),.RGHT(rght_ESC),.SS_A2D_n(SS_A2D_n),.SCLK_A2D(SCLK_A2D),
				.MOSI_A2D(MOSI_A2D),.MISO_A2D(MISO_A2D));


//// Instantiate Master UART (used to send commands to Copter) //////
CommMaster iMSTR(.clk(clk), .rst_n(rst_n), .RX(TX), .TX(RX),
                 .cmd(cmd), .data(data), .snd_cmd(snd_cmd),
			     .frm_snt(frm_snt), .rx_rdy(rx_rdy),
				 .rx_data(rx_data), .clr_frm_snt_out(clr_frm_snt_out));
				 


				 

initial begin

  //This is where you do the real work.
  //This section could be done as a bunch of calls to testing sub tasks contained in a separate file.
  
  //You might want to consider having several versions of this file that test several different
  //smaller things instead of having one huge test that runs forever.
  rst_n = 0;
  clk = 0;
  
  @(negedge clk)
  rst_n = 1;

  
  send_cmd(SET_PTCH, 16'h0001);
  send_cmd(SET_ROLL, 16'h0002);
  send_cmd(SET_THRST, 16'h0003);
end

task send_cmd;
	input [7:0] cmd_task;
	input [15:0] data_task;
	begin
		cmd = cmd_task;
		data = data_task;
		snd_cmd=1;
		$display("sending cmd, data  %h,%h",cmd_task,data_task);
		@(negedge clk);
		snd_cmd=0;
		@(negedge clk);
		@(posedge rx_rdy)
		$display("resp is %h", rx_data);
		$stop();
	end

endtask


always
  #10 clk = ~clk;

//`include "tb_tasks.v"	// maybe have a separate file with tasks to help with testing

endmodule	
