module LASER (
input CLK,
input RST,
input [3:0] X,
input [3:0] Y,
output reg [3:0] C1X,
output reg [3:0] C1Y,
output reg [3:0] C2X,
output reg [3:0] C2Y,
output reg DONE);

parameter GETDATA = 1;
parameter ITERATION = 2;
parameter STOP = 3;
parameter FINISH = 4;
parameter MASKING = 5;
parameter UNMASKING = 6;
reg item [0:255];
reg [7:0] position, positionRecord;
reg [4:0] count;
reg [2:0] state,nextState;
reg [5:0] target1sum, target2sum, sum, recalculateSum, totalsum;
reg [3:0] target1x, target1y, target2x, target2y;
reg finishGetdata, finishIteration, positionChange, targetChoose, finishMasking, finishUNMasking, gotoUNMasking, allStop;
wire n64, n50, n49, n48, n47, n46, n35, n34, n33, n32, n31, n30, n29, n19, n18, n17, n16, n15, n14, n13, n4, n3, n2, n1;
wire center, p64, p50, p49, p48, p47, p46, p35, p34, p33, p32, p31, p30, p29, p19, p18, p17, p16, p15, p14, p13, p4, p3, p2, p1;
wire n64_mask, n50_mask, n49_mask, n48_mask, n47_mask, n46_mask, n35_mask, n34_mask, n33_mask, n32_mask, n31_mask, n30_mask, n29_mask, n19_mask, n18_mask, n17_mask, n16_mask, n15_mask, n14_mask, n13_mask, n4_mask, n3_mask, n2_mask, n1_mask;
wire center_mask, p64_mask, p50_mask, p49_mask, p48_mask, p47_mask, p46_mask, p35_mask, p34_mask, p33_mask, p32_mask, p31_mask, p30_mask, p29_mask, p19_mask, p18_mask, p17_mask, p16_mask, p15_mask, p14_mask, p13_mask, p4_mask, p3_mask, p2_mask, p1_mask;
wire [3:0] level1, level2, level3, level4, level5, level6, level7, level8, level9;
wire [3:0] level1_mask, level2_mask, level3_mask, level4_mask, level5_mask, level6_mask, level7_mask, level8_mask, level9_mask;
wire [5:0] sum4x4, sum4x4_mask;
reg mask [0:255];
assign level1_mask = n64_mask;
assign level2_mask = n50_mask + n49_mask + n48_mask + n47_mask + n46_mask;
assign level3_mask = n35_mask + n34_mask + n33_mask + n32_mask + n31_mask + n30_mask + n29_mask;
assign level4_mask = n19_mask + n18_mask + n17_mask + n16_mask + n15_mask + n14_mask + n13_mask;
assign level5_mask = n4_mask  + n3_mask  + n2_mask  + n1_mask  + center_mask + p1_mask + p2_mask + p3_mask + p4_mask;
assign level6_mask = p19_mask + p18_mask + p17_mask + p16_mask + p15_mask + p14_mask + p13_mask;
assign level7_mask = p35_mask + p34_mask + p33_mask + p32_mask + p31_mask + p30_mask + p29_mask;
assign level8_mask = p50_mask + p49_mask + p48_mask + p47_mask + p46_mask;
assign level9_mask = p64_mask;

assign level1 = n64;
assign level2 = n50 + n49 + n48 + n47 + n46;
assign level3 = n35 + n34 + n33 + n32 + n31 + n30 + n29;
assign level4 = n19 + n18 + n17 + n16 + n15 + n14 + n13;
assign level5 = n4  + n3 + n2 + n1 + center + p1 + p2 + p3 + p4;
assign level6 = p19 + p18 + p17 + p16 + p15 + p14 + p13;
assign level7 = p35 + p34 + p33 + p32 + p31 + p30 + p29;
assign level8 = p50 + p49 + p48 + p47 + p46;
assign level9 = p64;

reg [7:0] anotherTarget;

wire [6:0] range [0:23];

assign range[0] = 64;
assign range[1] = 50;
assign range[2] = 49;
assign range[3] = 48;
assign range[4] = 47;
assign range[5] = 46;
assign range[6] = 35;
assign range[7] = 34;
assign range[8] = 33;
assign range[9] = 32;
assign range[10] = 31;
assign range[11] = 30;
assign range[12] = 29;
assign range[13] = 19;
assign range[14] = 18;
assign range[15] = 17;
assign range[16] = 16;
assign range[17] = 15;
assign range[18] = 14;
assign range[19] = 13;
assign range[20] = 4;
assign range[21] = 3;
assign range[22] = 2;
assign range[23] = 1;

assign center  = item[position - 0];
assign n64 = position[7:4] >= 4 && item[position - 64];                            
assign n50 = position[7:4] >= 3 && position[3:0] >= 2 && item[position - 50];      
assign n49 = position[7:4] >= 3 && position[3:0] >= 1 && item[position - 49];      
assign n48 = position[7:4] >= 3 && item[position - 48];                            
assign n47 = position[7:4] >= 3 && position[3:0] <= 15 - 1 && item[position - 47]; 
assign n46 = position[7:4] >= 3 && position[3:0] <= 15 - 2 && item[position - 46]; 
assign n35 = position[7:4] >= 2 && position[3:0] >= 3 && item[position - 35];      
assign n34 = position[7:4] >= 2 && position[3:0] >= 2 && item[position - 34];      
assign n33 = position[7:4] >= 2 && position[3:0] >= 1 && item[position - 33];      
assign n32 = position[7:4] >= 2 && item[position - 32];                            
assign n31 = position[7:4] >= 2 && position[3:0] <= 15 - 1 && item[position - 31]; 
assign n30 = position[7:4] >= 2 && position[3:0] <= 15 - 2 && item[position - 30]; 
assign n29 = position[7:4] >= 2 && position[3:0] <= 15 - 3 && item[position - 29]; 
assign n19 = position[7:4] >= 1 && position[3:0] >= 3 && item[position - 19];      
assign n18 = position[7:4] >= 1 && position[3:0] >= 2 && item[position - 18];      
assign n17 = position[7:4] >= 1 && position[3:0] >= 1 && item[position - 17];      
assign n16 = position[7:4] >= 1 && item[position - 16];                            
assign n15 = position[7:4] >= 1 && position[3:0] <= 15 - 1 && item[position - 15]; 
assign n14 = position[7:4] >= 1 && position[3:0] <= 15 - 2 && item[position - 14]; 
assign n13 = position[7:4] >= 1 && position[3:0] <= 15 - 3 && item[position - 13]; 
assign n4  = 					   position[3:0] >= 4 && item[position -  4];      
assign n3  =                       position[3:0] >= 3 && item[position -  3];      
assign n2  =                       position[3:0] >= 2 && item[position -  2];      
assign n1  =                       position[3:0] >= 1 && item[position -  1];      

assign p1  =                            position[3:0] <= 15 - 1 && item[position +  1]; 
assign p2  = 	                        position[3:0] <= 15 - 2 && item[position +  2]; 
assign p3  = 							position[3:0] <= 15 - 3 && item[position +  3]; 
assign p4  = 							position[3:0] <= 15 - 4 && item[position +  4]; 
assign p19 = position[7:4] <= 15 - 1 && position[3:0] <= 15 - 3 && item[position + 19]; 
assign p18 = position[7:4] <= 15 - 1 && position[3:0] <= 15 - 2 && item[position + 18]; 
assign p17 = position[7:4] <= 15 - 1 && position[3:0] <= 15 - 1 && item[position + 17]; 
assign p16 = position[7:4] <= 15 - 1 && item[position + 16];                            
assign p15 = position[7:4] <= 15 - 1 && position[3:0] >= 1 && item[position + 15];      
assign p14 = position[7:4] <= 15 - 1 && position[3:0] >= 2 && item[position + 14];      
assign p13 = position[7:4] <= 15 - 1 && position[3:0] >= 3 && item[position + 13];      
assign p35 = position[7:4] <= 15 - 2 && position[3:0] <= 15 - 3 && item[position + 35]; 
assign p34 = position[7:4] <= 15 - 2 && position[3:0] <= 15 - 2 && item[position + 34]; 
assign p33 = position[7:4] <= 15 - 2 && position[3:0] <= 15 - 1 && item[position + 33]; 
assign p32 = position[7:4] <= 15 - 2 && item[position + 32];                            
assign p31 = position[7:4] <= 15 - 2 && position[3:0] >= 1 && item[position + 31];      
assign p30 = position[7:4] <= 15 - 2 && position[3:0] >= 2 && item[position + 30];      
assign p29 = position[7:4] <= 15 - 2 && position[3:0] >= 3 && item[position + 29];      
assign p50 = position[7:4] <= 15 - 3 && position[3:0] <= 15 - 2 && item[position + 50]; 
assign p49 = position[7:4] <= 15 - 3 && position[3:0] <= 15 - 1 && item[position + 49]; 
assign p48 = position[7:4] <= 15 - 3 && item[position + 48];                            
assign p47 = position[7:4] <= 15 - 3 && position[3:0] >= 1 && item[position + 47];      
assign p46 = position[7:4] <= 15 - 3 && position[3:0] >= 2 && item[position + 46];      
assign p64 = position[7:4] <= 15 - 4 && item[position + 64];                            

assign center_mask  = item[position - 0] && mask[position];
assign n64_mask = n64 && mask[position - 64];
assign n50_mask = n50 && mask[position - 50];
assign n49_mask = n49 && mask[position - 49];
assign n48_mask = n48 && mask[position - 48];
assign n47_mask = n47 && mask[position - 47];
assign n46_mask = n46 && mask[position - 46];
assign n35_mask = n35 && mask[position - 35];
assign n34_mask = n34 && mask[position - 34];
assign n33_mask = n33 && mask[position - 33];
assign n32_mask = n32 && mask[position - 32];
assign n31_mask = n31 && mask[position - 31];
assign n30_mask = n30 && mask[position - 30];
assign n29_mask = n29 && mask[position - 29];
assign n19_mask = n19 && mask[position - 19];
assign n18_mask = n18 && mask[position - 18];
assign n17_mask = n17 && mask[position - 17];
assign n16_mask = n16 && mask[position - 16];
assign n15_mask = n15 && mask[position - 15];
assign n14_mask = n14 && mask[position - 14];
assign n13_mask = n13 && mask[position - 13];
assign n4_mask  = n4 && mask[position -  4];
assign n3_mask  = n3 && mask[position -  3];
assign n2_mask  = n2 && mask[position -  2];
assign n1_mask  = n1 && mask[position -  1];

assign p1_mask  = p1 && mask[position +  1];
assign p2_mask  = p2 && mask[position +  2];
assign p3_mask  = p3 && mask[position +  3];
assign p4_mask  = p4 && mask[position +  4];
assign p19_mask = p19 && mask[position + 19];
assign p18_mask = p18 && mask[position + 18];
assign p17_mask = p17 && mask[position + 17];
assign p16_mask = p16 && mask[position + 16];
assign p15_mask = p15 && mask[position + 15];
assign p14_mask = p14 && mask[position + 14];
assign p13_mask = p13 && mask[position + 13];
assign p35_mask = p35 && mask[position + 35];
assign p34_mask = p34 && mask[position + 34];
assign p33_mask = p33 && mask[position + 33];
assign p32_mask = p32 && mask[position + 32];
assign p31_mask = p31 && mask[position + 31];
assign p30_mask = p30 && mask[position + 30];
assign p29_mask = p29 && mask[position + 29];
assign p50_mask = p50 && mask[position + 50];
assign p49_mask = p49 && mask[position + 49];
assign p48_mask = p48 && mask[position + 48];
assign p47_mask = p47 && mask[position + 47];
assign p46_mask = p46 && mask[position + 46];
assign p64_mask = p64 && mask[position + 64];
assign sum4x4_mask = level1_mask + level2_mask + level3_mask + level4_mask + level5_mask + level6_mask + level7_mask + level8_mask + level9_mask;
assign sum4x4 = level1 + level2 + level3 + level4 + level5 + level6 + level7 + level8 + level9;

//aim: to find two point to maximize cover point
//my aim: finish it in 150 lines
always@(state, finishGetdata, finishIteration,finishUNMasking, finishMasking, gotoUNMasking)begin
	case(state)
		GETDATA:
			if(finishGetdata == 1)
				nextState = ITERATION;
			else
				nextState = GETDATA;
		ITERATION:
			if(gotoUNMasking == 1)
				nextState = UNMASKING;
			else if(finishIteration == 1)
				nextState = STOP;
			else 
				nextState = ITERATION;
		UNMASKING:
			if(finishUNMasking)
				nextState = MASKING;
			else
				nextState = UNMASKING;
		MASKING:
			if(finishMasking)
				nextState = ITERATION;
			else
				nextState = MASKING;
		default:
			nextState = 1;
	endcase
end
always@(posedge CLK)begin
	if(RST)
		state = 1;
	else
		state = nextState;
end
integer i;
always@(posedge CLK)begin
	finishGetdata <= 0;
	finishIteration <= 0;
	finishMasking <= 0;
	finishUNMasking <= 0;
	gotoUNMasking <= 0;
	DONE <= 0;
	
	if(RST)begin
		totalsum <= 0;
		count <= 0;
		target1sum <= 0;
		target2sum <= 0;
		targetChoose <= 0;
		positionChange <= 0;
		anotherTarget = 0;
		DONE <= 0;
		sum <= 0;
		recalculateSum <= 0;
		position <= 0;
		allStop <= 0;
		for(i = 0;i <= 255 ;i =i + 1)begin
			item[i] <= 0;
			mask[i] <= 1;
		end
	end
	else
		case(state)
			GETDATA:begin
				position <= position + 1;
				item[X + 16 * Y] <= 1;
				if(position == 40)begin
					position <= 0;
					finishGetdata <= 1;
				end
				else begin
					position <= position + 1;
					finishGetdata <= 0;
				end
					
			end
			ITERATION:begin
				//iteration: first step is to find the maximum sum as first point
				if(targetChoose == 0)begin
					if((sum4x4_mask + target2sum) > totalsum)begin
						totalsum <= sum4x4_mask + target2sum;
						C1X <= position[3:0];
						C1Y <= position[7:4];
						
						
						positionRecord <= position;
						positionChange <= 1;
							
						
					end
					else if((sum4x4_mask + target2sum) == totalsum)begin
						C1X <= position[3:0];
						C1Y <= position[7:4];
						if(position > positionRecord)begin
							positionRecord <= position;
							positionChange <= 1;
						end
						else begin
							positionRecord <= positionRecord;
							positionChange <= positionChange;
						end
						
					end
					else begin
						C1X <= C1X;
						C1Y <= C1Y;
					end
					
					if(position == 255)begin
						positionChange <= 0;
						position <= C1X + 16 * C1Y;
						
						if(positionChange == 0&& allStop)begin
							finishIteration <= 1;
							
						end
						else if(positionChange == 0)begin
							
							allStop <= 1;
							targetChoose <= 1;
							gotoUNMasking <= 1;
						end
						else begin
							allStop <= 0;
							targetChoose <= 1;
							gotoUNMasking <= 1;
						end
					end
					else
						position <= position + 1;
				end
				else begin
					if((sum4x4_mask + target1sum) > totalsum)begin
						C2X <= position[3:0];
						C2Y <= position[7:4];
						totalsum <= sum4x4_mask + target1sum;
						positionRecord <= position;
						positionChange <= 1;
						
					end
					else if((sum4x4_mask + target1sum) == totalsum)begin
						C2X <= position[3:0];
						C2Y <= position[7:4];
						if(position > positionRecord)begin
							positionRecord <= position;
							positionChange <= 1;
						end
						else begin
							positionRecord <= positionRecord;
							positionChange <= positionChange;
							
						end
					end
					else begin
						C2X <= C2X;
						C2Y <= C2Y;
					end
					
					if(position == 255)begin
						position <= C2X + 16 * C2Y;
						positionChange <= 0;
						if(positionChange == 0&& allStop)begin
							finishIteration <= 1;
						end
						else if(positionChange == 0)begin
							
							allStop <= 1;
							targetChoose <= 0;
							gotoUNMasking <= 1;
						end
						else begin
							allStop <= 0;
							
							targetChoose <= 0;
							gotoUNMasking <= 1;
						end
					end
					else
						position <= position + 1;
				end
			end
			MASKING:begin
				//make masking
				if(targetChoose == 1)
					anotherTarget = C1X + 16 * C1Y;
				else
					anotherTarget = C2X + 16 * C2Y;
				mask[anotherTarget - range[count]] <= 0;
				mask[anotherTarget + range[count]] <= 0;
				mask[anotherTarget] <= 0;
				
				if(count < 23)
					count <= count + 1;
				else begin
					count <= 0;
					sum <= 0;
					if(targetChoose == 1)begin
						target1sum <= sum4x4;
						target2sum <= target2sum;
						
					end
					else if(targetChoose == 0)begin
						target2sum <= sum4x4;
						target1sum <= target1sum;
						
						
					end
					else begin
						target1sum <= target1sum;
						target2sum <= target2sum;
						
					end
						
					finishMasking <= 1;
					position <= 0;
				end
				
			end
			UNMASKING:begin
				//make unmasking
				mask[anotherTarget - range[count]] <= 1;
				mask[anotherTarget + range[count]] <= 1;
				mask[anotherTarget] <= 1;
				if(count < 23)
					count <= count + 1;
				else begin
					count <= 0;
					finishUNMasking <= 1;
					
				end
			
			end
			STOP:begin
				DONE <= 1;
				
				totalsum <= 0;
				count <= 0;
				target1sum <= 0;
				target2sum <= 0;
				targetChoose <= 0;
				positionChange <= 0;
				anotherTarget = 0;
				sum <= 0;
				recalculateSum <= 0;
				position <= 0;
				allStop <= 0;
				for(i = 0;i <= 255 ;i =i + 1)begin
					item[i] <= 0;
					mask[i] <= 1;
				end
				
			end
		endcase
end
endmodule


