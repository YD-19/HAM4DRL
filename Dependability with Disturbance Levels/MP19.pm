dtmc
//version:V3 18/08/2021 by XZ.

//Listing all transition probabilities as variables.
const double p_G_B1 = 0.060149783208514;
const double p_G_B2 = 0.00370516357903035;
const double p_G_B3 = 0.000157666535277887;
const double p_G_C = 0;
formula p_G_G = 1-p_G_B1-p_G_B2-p_G_B3-p_G_C;//the sum of all outgoing tran. prob. is 1

const double p_B1_G = 0.042429917481126;
const double p_B1_B2 = 0.133668871071575;
const double p_B1_B3 = 0.0028091531573711;
const double p_B1_C = 0;
formula p_B1_B1 = 1-p_B1_G-p_B1_B2-p_B1_B3-p_B1_C;

const double p_B2_G = 0.00450634985661614;
const double p_B2_B1 = 0.0731257681278165;
const double p_B2_B3 = 0.120442441622286;
const double p_B2_C = 0.00122900450634986;
formula p_B2_B2 = 1-p_B2_G-p_B2_B1-p_B2_B3-p_B2_C;

const double p_B3_G = 0.000616903146206046;
const double p_B3_B1 = 0.00445541161148811;
const double p_B3_B2 = 0.23449174035232;
const double p_B3_C = 0.00918500239906779;
formula p_B3_B3 = 1-p_B3_G-p_B3_B1-p_B3_B2-p_B3_C;

module FailureProcByDistance
//This is the "failure process DTMC" abstracted based on the distance to obstacles
	s: [0..4] init 0; 	
	// 0 - G, ideal distance to the obstacle, "golden route".
	// 1 - B1, a bit close to the obstacle, benign failue level 1.
	// 2 - B2, somewhat close to the obstacle, benign failure level 2.
	// 3 - B3, too close to the obstacle, benign failure level 3.
	// 4 - C, crash into the obstacle, catastrophic failure.
	[tran] s=0 -> p_G_G:(s'=0)+p_G_B1:(s'=1)+p_G_B2:(s'=2)+p_G_B3:(s'=3)+p_G_C:(s'=4);
	[tran] s=1 -> p_B1_G:(s'=0)+p_B1_B1:(s'=1)+p_B1_B2:(s'=2)+p_B1_B3:(s'=3)+p_B1_C:(s'=4);
	[tran] s=2 -> p_B2_G:(s'=0)+p_B2_B1:(s'=1)+p_B2_B2:(s'=2)+p_B2_B3:(s'=3)+p_B2_C:(s'=4);
	[tran] s=3 -> p_B3_G:(s'=0)+p_B3_B1:(s'=1)+p_B3_B2:(s'=2)+p_B3_B3:(s'=3)+p_B3_C:(s'=4);
	[tran] s=4 -> 1:(s'=4);
endmodule

const int avg_mission_length=560;//average mission length, e.g., 500 steps.

module MissionStage
// This is the "mission-stage DTMC" represents the main stages of the mission.
	k:[0..1] init 0;//0: mission not finished; 1: mission finished
	[tran] k=0 -> 1/avg_mission_length:(k'=1)+(1-1/avg_mission_length):(k'=0);
endmodule

rewards "step"
	s<5 & k=0 : 1; //reward of 1 to every state of the model, thus representing steps..
endrewards

rewards "deviation"
	s=0 : 0; // 0m deviation from the 'golden route' (non-risky distance to obstacles)
	s=1 : 1; // 1m deviation from the 'golden route'
	s=2 : 2; // 2m deviation from the 'golden route'
	s=3 : 3; // 3m deviation from the 'golden route'
	s=4 : 0; // No practical meaning when crash, thus we use 0 as a placeholder.
endrewards
