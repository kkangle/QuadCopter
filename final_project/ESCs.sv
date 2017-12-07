module ESCs(clk,rst_n,frnt_spd,bck_spd,lft_spd,rght_spd,motors_off,frnt,bck,lft,rght);

	input clk;		// our 50MHz clock from DE0-Nano
	input rst_n;	// from push button, goes to our rst_synch block
	input [10:0]frnt_spd,bck_spd,lft_spd,rght_spd;		
	input motors_off;

	localparam FRNT_OFF=10'h220;
	localparam BCK_OFF=10'h220;
	localparam LFT_OFF=10'h220;
	localparam RGHT_OFF=10'h220;

	output logic frnt,bck,lft,rght;

	logic [10:0] speed1,speed2,speed3,speed4;
	logic [9:0] off1,off2,off3,off4;
	
	assign speed1=frnt_spd&&~motors_off;
	assign speed2=bck_spd&&~motors_off;
	assign speed3=lft_spd&&~motors_off;
	assign speed4=rght_spd&&~motors_off;
	
	assign off1=FRNT_OFF&&~motors_off;
	assign off2=BCK_OFF&&~motors_off;
	assign off3=LFT_OFF&&~motors_off;
	assign off4=RGHT_OFF&&~motors_off;
	 
	
	
	///////////////////////////////////////////////////
	// Instantiate ESC_interface (which is the DUT) //
	/////////////////////////////////////////////////
	ESC_interface #(18'h3ffff)iDUT1(.clk(clk), .rst_n(rst_n), .OFF(off1), .SPEED(speed1), .PWM(frnt));
	ESC_interface #(18'h3ffff)iDUT2(.clk(clk), .rst_n(rst_n), .OFF(off2), .SPEED(speed2), .PWM(bck));
	ESC_interface #(18'h3ffff)iDUT3(.clk(clk), .rst_n(rst_n), .OFF(off3), .SPEED(speed3), .PWM(lft));
	ESC_interface #(18'h3ffff)iDUT4(.clk(clk), .rst_n(rst_n), .OFF(off4), .SPEED(speed4), .PWM(rght));

endmodule

