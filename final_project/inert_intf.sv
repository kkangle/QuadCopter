module inert_intf(clk,strt_cal,rst_n,INT,cal_done,vld,ptch,roll,yaw,SS_n,SCLK,MISO,MOSI);

input clk,rst_n; // 50MHz clk, active low asynch reset
input strt_cal;  // From cmd_cfg unit, tells inertial_integrator to start cal.
input INT; 		 // INT pin from inertial sensor, tells us a new measurement is ready

output logic cal_done; // assert when clpt calibration, to cmd_cfg
output logic vld; // output from internal SM, also an output to flght_cntrl.
output logic [15:0] ptch,roll,yaw; //  fusion corrected pitch, roll, and yaw of the quadcopter.

// in/out SPI interface to inertial sensor
output logic SS_n,SCLK,MOSI;
input MISO; 
logic [15:0] rd_data; 

logic [15:0] cnt;
logic wrt; // tell SPI to wrt
logic done; // from SPI, tell wrt done
logic [15:0] cmd; // cmd to SPI
logic C_P_H,C_P_L,C_R_H,C_R_L,C_Y_H,C_Y_L,C_AX_H,C_AX_L,C_AY_H,C_AY_L; // from SM to regs

// rate needed to be integrated to get ptch,roll,yaw 
logic [15:0] ptch_rt,roll_rt,yaw_rt,ax,ay;

// each 8 bits, so divide ptch.. into low nad high bytes
logic [7:0] ptch_H,ptch_L,roll_H,roll_L,yaw_H,yaw_L,AX_H,AX_L,AY_H,AY_L;  

// instantiate SPI and integrator 
inertial_integrator #(11) integrator(.clk(clk),.rst_n(rst_n),.strt_cal(strt_cal),.cal_done(cal_done),.vld(vld),
								.ptch_rt(ptch_rt),.roll_rt(roll_rt),.yaw_rt(yaw_rt),.ax(ax),.ay(ay),.ptch(ptch),.roll(roll),.yaw(yaw));

SPI_mstr16 SPI(.clk(clk),.rst_n(rst_n),.cmd(cmd),.wrt(wrt),.rd_data(rd_data),.SS_n(SS_n),.SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),.done(done));

assign ptch_rt = {ptch_H,ptch_L};
assign roll_rt = {roll_H,roll_L};
assign yaw_rt = {yaw_H,yaw_L};
assign ax = {AX_H,AX_L};
assign ay = {AY_H,AY_L};


// double flop INT for meta-stability reasons
logic INTff1, INTff2;
always@(posedge clk, negedge rst_n) begin
	if(!rst_n) INTff1 <= 1'b0;
	else INTff1 <= INT;
end

always@(posedge clk, negedge rst_n) begin
	if(!rst_n) INTff2 <= 1'b0;
	else INTff2 <= INTff1;
end


// holding regs
always@(posedge clk, negedge rst_n) begin
	if (!rst_n) ptch_L <= 0;
	else if(C_P_L) ptch_L <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) ptch_H <= 0;
	else if(C_P_H) ptch_H <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) roll_L <= 0;
	else if(C_R_L) roll_L <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) roll_H <= 0;
	else if(C_R_H) roll_H <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) yaw_L <= 0;
	else if(C_Y_L) yaw_L <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) yaw_H <= 0;
	else if(C_Y_H) yaw_H <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) AX_L <= 0;
	else if(C_AX_L) AX_L <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) AX_H <= 0;
	else if(C_AX_H) AX_H <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) AY_L <= 0;
	else if(C_AY_L) AY_L <= rd_data[7:0]; 
end

always@(posedge clk, negedge rst_n) begin
	if (!rst_n) AY_H <= 0;
	else if(C_AY_H) AY_H <= rd_data[7:0]; 
end



// time counter used for first state for waiting sensor to be rdy
// 16bits => 64k, 50MHz => 1kHz, wait 1ms 
always@(posedge clk, negedge rst_n) begin
	if (!rst_n) cnt <= 0;
	else cnt <= cnt + 1; 
end

// SM
typedef enum reg [3:0] {INIT1,INIT2,INIT3,INIT4,INIT5,WAIT,W_P_L,W_P_H,W_R_L,W_R_H,W_Y_L,W_Y_H,W_AX_L,W_AX_H,W_AY_L,W_AY_H} state_t;
state_t state, next_state;

always@(posedge clk, negedge rst_n) begin 
	if(!rst_n) state <= INIT1;
	else state <= next_state;
end

always_comb begin
	next_state = INIT1;
	wrt = 0;
	vld = 0; // also output from interface to outside
	C_P_H = 0;
	C_P_L = 0;
	C_R_H = 0;
	C_R_L = 0;
	C_Y_H = 0;
	C_Y_L = 0;
	C_AX_H = 0;
	C_AX_L = 0;
	C_AY_H = 0;
	C_AY_L = 0;
	cmd = 16'h00;

case(state)

	INIT1: begin  
		next_state = INIT1;
		if(&cnt) begin //cnt = 16'h1111
			$display("init1->init2");
			next_state = INIT2;
			wrt = 1;
			cmd = 16'h0D02; // follow spec of sensor: Enable interrupt upon data ready
		end
	end
	
	INIT2: begin  
		next_state = INIT2;
		if(done) begin 
			$display("init2->init3");
			next_state = INIT3;
			wrt = 1;
			cmd = 16'h1062; // Setup accel for 416Hz data rate, +/- 2g accel range, 100Hz LPF
		end
	end

	INIT3: begin  
		next_state = INIT3;
		if(done) begin 
			$display("init3->init4");
			next_state = INIT4;
			wrt = 1;
			cmd = 16'h1162; // Setup gyro for 416Hz data rate, +/- 125°/sec range.
		end
	end

	INIT4: begin  
		next_state = INIT4;
		if(done) begin 
			next_state = INIT5;
			wrt = 1;
			cmd = 16'h1460; // Turn rounding on for both accel and gyro
		end
	end
	
	INIT5: begin  
		next_state = INIT5;
		if(done) begin 
			next_state = WAIT; // no outputs
		end
	end
	
	WAIT: begin  
		next_state = WAIT;
		if(INTff2) begin 
			next_state = W_P_L;
			wrt = 1;
			cmd = 16'hA2xx;
		end
	end

	W_P_L: begin  
		next_state = W_P_L;
		if(done) begin 
			next_state = W_P_H;
			wrt = 1;
			cmd = 16'hA3xx;
			C_P_L = 1;
		end
	end

	W_P_H: begin  
		next_state = W_P_H;
		if(done) begin 
			next_state = W_R_L;
			wrt = 1;
			cmd = 16'hA4xx;
			C_P_H = 1;
		end
	end
	
	W_R_L: begin  
		next_state = W_R_L;
		if(done) begin 
			next_state = W_R_H;
			wrt = 1;
			cmd = 16'hA5xx;
			C_R_L = 1;
		end
	end
	
	W_R_H: begin  
		next_state = W_R_H;
		if(done) begin 
			next_state = W_Y_L;
			wrt = 1;
			cmd = 16'hA6xx;
			C_R_H = 1;
		end
	end
	
	W_Y_L: begin  
		next_state = W_Y_L;
		if(done) begin 
			next_state = W_Y_H;
			wrt = 1;
			cmd = 16'hA7xx;
			C_Y_L = 1;
		end
	end
	
	W_Y_H: begin  
		next_state = W_Y_H;
		if(done) begin 
			next_state = W_AX_L;
			wrt = 1;
			cmd = 16'hA8xx;
			C_Y_H = 1;
		end
	end
	
	W_AX_L: begin  
		next_state = W_AX_L;
		if(done) begin 
			next_state = W_AX_H;
			wrt = 1;
			cmd = 16'hA9xx;
			C_AX_L = 1;
		end
	end
	
	W_AX_H: begin  
		next_state = W_AX_H;
		if(done) begin 
			next_state = W_AY_L;
			wrt = 1;
			cmd = 16'hAAxx;
			C_AX_H = 1;
		end
	end
	
	W_AY_L: begin  
		next_state = W_AY_L;
		if(done) begin 
			next_state = W_AY_H;
			wrt = 1;
			cmd = 16'hABxx;
			C_AY_L = 1;
		end
	end
	
	W_AY_H: begin  
		next_state = W_AY_H;
		if(done) begin 
			next_state = WAIT;
			vld = 1;
			C_AY_H = 1;
		end
	end

	default next_state = INIT1;
	
endcase
end
	
endmodule