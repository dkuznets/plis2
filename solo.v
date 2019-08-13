module solo(a12mhz,svetodiod,adresbus,databus,cs,oe,bls0we,adresmem,datamem,oemem,wemem,
			mreset,mfield,mdata,mfsync,mlsync,mclock,clockADC,
			acdata,foto,ingo,ready,go);
parameter lineclk=320/2; //����� �������� � ������/2 ����� clk � ������ - ���������� ����� ��������� � ������
parameter line=256;//����� ����� � ������� - � ���� ��������
parameter psw=33'b010000010010000000000000000000010; // ����� ��������� ��� ������ �������
parameter block=300/10;// ����� ������������ xxx/10 ns ���������� �������� ��� ������ � FIFO xxx ������ ���� ������ 3 !!!
//parameter dlinay=261;// a?aiy eioaa?e?iaaiey i?e aeeiiie auaa??ea aac auno?aea 261-118+50 =
//parameter dlinaImit=361;
//parameter dlin=183;// iauee ei?ioeee ia?eia eioaa?e?iaaiea 
parameter dlinay=10000;// a?aiy eioaa?e?iaaiey i?e aeeiiie auaa??ea aac auno?aea 261-118+50 =
parameter dlinaImit=10000;// aey aeeiiie auaa??ee aey eaeea?iaee
parameter dlin=25000;// iauee ei?ioeee ia?eia eioaa?e?iaaiea 
//������� �� ������� 
output mreset,mfield,mdata,mfsync,mlsync,mclock;
reg mreset,mfield;
wire mclock,mfsync,mlsync,mdata;
//������� � ADC � ��  ADC
output	clockADC;
reg clockADC;
input	[12:0]	acdata;
reg [12:0] adcreg;
// ������ � ���������
input	foto;
//���������� �������� ���������
input a12mhz;	//������� ������� ��� PLL
output svetodiod; // ������ �����������
wire svetodiod;
wire a96mhz;	//�������� ������� PLL - �������� �������� �������
wire a192mhz;	// ������ �������� �������
wire plllocked;	// ������� ������������
reg [13:0] div9600; // 100 us ��������
reg [30:0] timer; // ������� �������
reg sttimer; // ����� � ���������� ������ ������� ����������� RKS[6]
wire [30:0] datafifo;//������ �� fifo �������
wire timerempty,timerfull; //fifo ������� ������ � �����������
reg [30:0] REGTIMER; // ������� ������� ��������
wire autovistrel;// ��������� ������ �� foto � arm ��������
reg vistrel; // ������� ��������� ������� �� ����� ���������� �������
reg stvistrel;
reg kadrvistrel; // ������� �������� ��� �����
reg kadrvi;// ������� �������� � clock ������ 96MHz
reg intakadr,intbkadr,intckadr; //���� ������ ������� � �� ����������� ���� � ����������� �� ����
reg intckadrsecond; // ����� ������ ���� ����� ���������� �������� � ���� intckadr
reg intbkadrsecond; // ����� ������ ���� ����� �������� � ���������� ���������
reg intbkadrthree;// ������ ���� ����� ��������
output go; //������� ��������� ��������
reg go; // ������ ������� ������� - ����� ������� ������ ������� =1
input ingo; // ������� ������ �������������
wire ingo;
wire rksgo ; //������ ������� RKS[0] � ������ ������� �� U37 73 pin -> �� U39 4 pin
reg firstslave;// ������������� ������� ������� �� ������� � slave
reg gostart; // ������ go ������������ � clock �����
reg resetgo; // ����� ������� go ������������ ����� ������ �� ������� ���������� �����
reg waitgo; //������� � �������� �����
reg imit; //������� �������� �������� ��������������� �� ARM
reg [18:0] dlinatime; // ������� ������������ ��� ������� ����� 
reg resvistrel; // ����� ��������� vistrel
reg [15:0] waittick;
reg waitkadr;
reg fixstartkadr; // ������������� ������� � ������ ��������������
reg firsttimer;// ������ ������� � ���������� ������
reg fixtimer;// ����� ������������� � ����� �������� ������ �� ��������� ������� ������
reg [3:0] fifowait;// ������������� � ������� a96MHz
reg readfifo; // ������ �� arm ��� ������ fifo
reg getfifo;// ������� ��������� ������ �� ������ fifo
reg stgetfifo;// ������� � �������� �����
reg resgetfifo;// ��������� ����� ��������� ������ �� fifo
reg wrfifo;// ���������� ������ � fifo timer
reg rdfifo;// ���������� ������ �� fifo timer
wire flash,flasha; //���������� ������ �������� ��� �����
output ready;// ������ �� ���������� ������ ����� � ������ �������
reg ready;// ���������� ������������ �������� ������ ���������� ��������� �����

//��������� ��� ������ � ������������ �� fifo ���� ������
reg wrram;// ������ � fifo �� ��� ���� ������ cndata � linecount
reg rdram; // ������ �� fifo ��������� ���� ������
wire ramempty; //fifo �����
wire ramfull; // fifo ����������� � ���� ������������ ����� �� ������ � ��� ���� ��������� ��������
wire [17:0] ramfifo;// ������ �� fifo ������
reg getfiforam,readfiforam,resgetfiforam,stgetfiforam,fifowaitram;

//���������� �������� ������ ������� � ���
reg [3:0] faza; //���� ������������ ������� a96mhz
reg clocka;// clock ������� ����� ������ �� �������
reg [18:0] kodclocka; // ����� clock ������� ������ �������������� ��������(�������� ���������)
reg mfclocka;//����� �� ������� ������� frame
reg mfsynca,mlsynca; 
reg [32:0] sdvigpsw;
reg [32:0] sdvig; // ������� ���������� ��� �������� � ��������� ������
reg [10:0] cndata;// ������� ��������� clock � ������ -����� �������
reg [8:0] linecount; // ����� ������ �����
reg [10:0] cnreg;
reg [8:0] linereg;
reg testimit;// ������ �������� ����������� ���� ��� �������

// ���������� ������ ������
inout [7:0] datamem;
reg [7:0] datamem;
inout [7:0] databus;
reg [7:0] databus;

input [5:0] adresbus;
input cs,oe,bls0we;

output oemem,wemem;
reg oemem,wemem;
output [18:0]  adresmem;
reg [18:0] adresmem;

reg wememARM,oememARM;// ������� ������ ������ �� ������ �� ARM
reg wememADC; // ������ ������ �� ���� �� ADC oemem=1 ���������
reg [18:0]	adrreg; // ������� ������ �� ������ ����� �������� �� ARM
reg [2:0]	wait192;
reg read,write,resadd;
reg [18:0]	adrregADC; // ������� ������ �� ������ ����� �������� �� ADC
reg [30:0] temptimer; // ��������� ����������

//end ���������� ������ ������
// ���������� ������ ��������
reg adrRKS,adrRKSMEM,adrDATA,adrTIMER0,adrTIMER1,adrTIMER2,adrTIMER3,adrFIFO0,adrFIFO1,adrFIFO2,adrFIFO3;
reg adrPOROG0,adrPOROG1;
reg adrRAM0,adrRAM1,adrRAM2,adrRAM3; //�������� fiforam �������� ������ POROG
reg adrRKSRAM;
reg [7:0] RKS;
reg [7:0] RKSMEM;
reg [9:0] POROG;

// ����� ���������
pllgen mypll(.inclk0(a12mhz),.c0(a96mhz),.c1(a192mhz),.locked(plllocked)); // PLL ��� ��������� ������� ������� � �������� ��������
fifotimer myfifotimer(.clock(a96mhz),.data(temptimer),.rdreq(rdfifo),.wrreq(wrfifo),.empty(timerempty),.full(timerfull),.q(datafifo)); //fifo ��� ������ timer ���������
myfiforam fiforam(.clock(a96mhz),.data({cnreg[8:0],linereg[8:0]}),.rdreq(rdram),.wrreq(wrram),.empty(ramempty),.full(ramfull),.q(ramfifo)); // fifo ��� ������ �������� ���� ������
//������� ������ � ������ � FIFO � �������� ������� �������� � �������� � ����
always @(posedge a96mhz)
	begin
	if(RKS[6]==1)sttimer<=1; //��������� � �������� �����
	else sttimer<=0;
	if (sttimer==1)begin timer<=0;div9600<=0; temptimer<=0; end //RKS[6]==1 ���������� ������ � 0. ������ 0 �������� � ����� �������
		else begin if(div9600==9599)begin
					 div9600<=0;
					 if(timer==863999999)begin timer<=0; end
						else begin timer<=timer+1; temptimer<=timer; end
					end
		else begin div9600<=div9600+1; end
			end
// ��������� ����� �������� �������
	if(kadrvistrel==1)waitkadr<=1;
		else waitkadr<=0;
	if(waitkadr==1 && firsttimer==1)begin REGTIMER<=temptimer; fixtimer<=1; end // ������ ������������� � ��������� �� ������� ����� ��������
		else fixtimer<=0;

	if(vistrel==1)stvistrel<=1;// ��������� � ��������  ����� a96MHz
	if(stvistrel==1)waittick<=waittick+1;//���������� 1 clock 96MHz ��� �������� � ����� � block clock ��� ���������� �������� �� ���������
		else waittick<=0;
	if(waittick==block)begin resvistrel<=1;stvistrel<=0; end //(����� ���������� ���������) ������� ��������� � ���������� vistrel<=0 ����� resvistrel<=1 
				else resvistrel<=0;
// ������ � fifo ������� ��������
	if(waittick==1 )wrfifo<=1; // ���������� ������ timer � fifo
		else wrfifo<=0;
// ������ ������ �� fifotimer
	if(getfifo==1)stgetfifo<=1;// ��������� � �������� ����� a96MHZ - ����� �� ���������� ��� ��� ������� fifowait 1 ��� !!!
	if(stgetfifo==1)fifowait<=fifowait+1;
		else fifowait<=0;
	if(fifowait==1)begin rdfifo<=1; resgetfifo<=1; stgetfifo<=0;end // ������ �� fifo
		else begin rdfifo<=0; resgetfifo<=0; end
// ������ fiforam  �� �������� � fifotimer

	if(getfiforam==1)stgetfiforam<=1;// ��������� � �������� ����� a96MHZ - ����� �� ���������� ��� ��� ������� fifowait 1 ��� !!!
	if(stgetfiforam==1)fifowaitram<=fifowaitram+1;
		else fifowaitram<=0;
	if(fifowaitram==1)begin rdram<=1; resgetfiforam<=1; stgetfiforam<=0;end // ������ �� fifo
		else begin rdram<=0; resgetfiforam<=0; end

	end
//������� ������ �������
// ������� �� �������
assign mclock=~clocka; //clock �� ������� �� ������� ���� ����� ��������
assign mfsync=~mfsynca;//�� ������� �������� ����� �������� ��� ����
assign mlsync=~mlsynca;//�� ������� �������� ����� ��������
assign mdata=~sdvigpsw[32]; // ������ ������ ������ � �������� ����� ������� ��������

always @(posedge a96mhz)
	begin
	if(kadrvistrel==1)kadrvi<=1;//��������� � clock ����� ������� ������
		else kadrvi<=0;
	if(go==1 && rksgo==0)waitgo<=1; //��������� � �������� ����� ����� ���� ��� ����������� RKS[0] =0->1->0
		else  begin waitgo<=0; gostart<=0; end
	if (waitgo==1)begin gostart<=1; end 
	if(imit==1 && kodclocka==80 )begin testimit<=1; dlinatime<=dlinaImit; end//�������� �������� � ���� B - ����� ������� ��������
		else testimit<=0; 
	if(rksgo==1)begin fixstartkadr<=0; intakadr<=0; intbkadr<=0; intckadr<=0; intckadrsecond<=0; 
					intbkadrsecond<=0; intbkadrthree<=0; adrregADC<=0; linecount<=0;sdvig<=0; cndata<=0; end //����� �� ������ 1 � ������� RKS[0]
	if(gostart==1)	begin //���� ��������� �������� ������� go==1
		if(faza==9)faza<=0;// ������� ������� �� 10 
			else faza<=faza+1; // faza ������ ����������� clock faza==0 ������������ ��������� clock
// ����������� ������ � ����� ������ ���� �� ��� �� 3 ������ � ����� 3 �������+3 ����� ADC
//                        ������ ������ � ������ ������ ADC        ������  2 �� 242 �����
				if(cndata>=11 && cndata<=329 && linecount>=4 && linecount<=258 && faza>=7 && faza<=9)wememADC<=0; //��������� we � ������
					else wememADC<=1;
// ������ � FIFORAM ������������ �  ������� � ������ ���� �������� POROG[9:0]<=acdata[11:2]
				if(cnreg>=11 && cnreg<=329 && linereg>=4 && linereg<=258 && faza==9
				 && (POROG<=~adcreg[11:2]))wrram<=1; //��������� we � ������
					else wrram<=0;
					
					// � ������ ������ �������� ����������� ����� �� ������ ���������� ��� ������� ������� ������
				if(cndata>=11 && cndata<=329 && linecount>=4 && linecount<=258 && faza==2)adrregADC<=adrregADC+1; 
// clockADC ������� ������������ clock ������� �� 2 �����=20 ��
				if(faza>=2 &&  faza<=6) clockADC<=1;// ��������� clock ADC
					else clockADC<=0;
				if(faza==6)begin adcreg=acdata; cnreg<=cndata; linereg<=linecount; end
//�������������� ����������� ������� ��� ��������� �������� � faza=0
		if(faza==0 )begin
				//  ��� ��������                         
				if(((kodclocka>=dlin) && (intakadr==0 && intbkadr==0 && intckadrsecond==0)) || 
					((kodclocka>=dlinatime)&&(intbkadr==1)&&(intbkadrthree==0))) // ���������� �������� ��� ���������� ���� � ����� ��� ��������
							begin
							kodclocka<=0;//���� �������� ��� �� �������� ������
							clocka<=0;
							// ������� ������ ���� � ��������� ������������ ��������
							ready<=1;// ����� � ��������� ������������ -> ����� ��������� ����� ��������
							if(intckadr==1)intckadrsecond<=1; //���������� ��� �������� � ���������� ������� ����� � ���� C
							if(intbkadr==1)intbkadrsecond<=1; //���������� ��� �������� � ���������� ������� ����� � ���� B
							if(intbkadrsecond==1)intbkadrthree<=1;// ���������� ������� ����� � ������ ������ 3 ����� � ���� B
							end
					else begin kodclocka<=kodclocka+1; clocka<=clocka+1;  end//�������� ��������� �������� � ������ ����
				if(kodclocka>=0 && kodclocka<=117)mfsynca<=1; //������������ MFSYNC ������ 59 clocks �� ��������� ������ clock
					else mfsynca<=0;
// ������������ �������� ��������� ������ �����
				if(kodclocka<=(line+2)*(lineclk+16)*2 && kodclocka>=(1+sdvig) && kodclocka<=(2+sdvig))mlsynca<=1; // ��������� mlsynca
					else mlsynca<=0;
				if((kodclocka >=52) && (intakadr==0) && (intckadrsecond==0) && (intbkadrthree==0)) //26 clocks �� MFSYNC � ������ reset
					begin
					if(kodclocka[0]==0)begin sdvigpsw[32:1]<=sdvigpsw[31:0];sdvigpsw[0]<=0; end
					end
					else sdvigpsw=psw;// ������ �����		
// ���� ������� �� ��������� ����
// ������� ������ � ������� ������ ����� ������ � clock
		//����� ����

				if(kodclocka==(3+sdvig))linecount<=linecount+1;// ������ � 1 �� ..
				if(cndata>=((lineclk+16)*2-1))begin cndata<=0; end  // ������� ����� ������ ��� ����� ������
					else cndata<=cndata+1;// ������� ����� ������� � ������ � 0 �� ..
				if(cndata==(lineclk*2+12))sdvig<=sdvig+(lineclk+16)*2; //� ����� ������ ���������� ����� ������� kodclocka
			end
	end
//���� ������� �������� � ��������� ������� ��������������
//     ������ 25 clock                         �� ������� ������ fsync
	if((0<=kodclocka && kodclocka<=50) && (kadrvi==1) && fixstartkadr==0)
/*		begin intakadr<=1; end //������� ����� � ���� �� ������ ������ ����� ����� ������ ���� - ��� ������ �������*/
		begin intbkadr<=1; end //��� slave � ���� A ���� ���� C � ������� - ������� ������ ���� B
//���� B - ������� ��������� � ������ ������ � �������� �������� ���� - ��� ��������
	if((51<=kodclocka && kodclocka<=116) && (kadrvi==1) && fixstartkadr==0)
		begin intbkadr<=1; end //������� ����� � ������ ������ - ����� ���������� ���� � ������� ��� ������ ���� ����� ������������� ���� ������
	if((117<=kodclocka && kodclocka<=dlin) && (kadrvi==1) && fixstartkadr==0)
		begin intckadr<=1; end //������� ����� � �������������� � ����� ������ ������ ����� ������ ���������� �����
	if(kadrvi==1) fixstartkadr<=1; // ��� ������ ������ ������� ��������� ���� �� �������������� ��� ���!!
// ��������� ������ ���� ������ ������ �� ��������� ����
	if(kodclocka==((line+2)*(lineclk+16)*2+200+1))begin resetgo<=1; dlinatime<=dlinay; ready<=0; end //������� ���� � ������������ �� ���������� �������
		else resetgo<=0;
//���� ������� ���� �� ������ ��������� ���������	
	if(resetgo==1)begin
			kodclocka<=0;
			clocka<=0;
			end
	if(kodclocka==0) // ���� �� � ������ ����� �� ������ ��������� ���� ����������
		begin
		sdvig<=0;cndata<=0;linecount<=0;adrregADC<=0;
		end
	// ���� ������� �� reset �� ���� ������
// ���������� ����� � ������ - ����������� ������� ��� ���������� ���� �������
// ���������� ��� ����� ������ ����� (lineclk+16)*2   ������ 16 ������ clock � ����� ������
// � ����� ����� ����� (line+2)*(lineclk+16)*2+200 ������ 100 ������ clock � ����� �����
end
//�������� �������� �� ARM
//������ ���� �� ��������� ���� �� ARM ������ 1 � -> RKS[1]
always @(posedge bls0we or posedge resvistrel)
		if(resvistrel==1)imit<=0;
		else if(adrRKS==1 && cs==0 && databus[1]==1)imit<=1;
//or (autovistrel,foto,imit);//����� ����� �� �������� � �� ��������
or (autovistrel,foto,testimit);// ��� ���������� �������� �������� �������� � ���� B ����� ������� ��������
or (flasha,foto,testimit); // �������� ������� ��� ����� ��������� ������ ����� ready
and (flash,flasha,ready);// ������� ������ ������ ����� ������ ready==1 ���� ������� ���� ������
// ��� ������
//�� ������� ������� ����������� ������� � �� ������� �������� ������� ��������������� � ����� ���� �� ����� �� ������������
always @(posedge flash or posedge rksgo) // ������ ��� ������ ������� � ���������
	if(rksgo==1 )kadrvistrel<=0;  
		else kadrvistrel<=1;
//��� �������
//��������� �������� vistrel �� ���������� ������� � ����� ����� ���������
always @(posedge autovistrel or posedge resvistrel)
			if(resvistrel==1)vistrel<=0;
				else vistrel<=1;//��������� ������� � ���� �����������
// ������ ������ �� fifoRAM �� ARM ������ ������ 4 ��� � �������������� getfiforam ������� ����� �������� resgetfiforam
always @(posedge bls0we or posedge getfiforam)
	if(getfiforam==1)readfiforam<=0;
		else begin if(adrRKSRAM==1 && cs==0 && databus[4]==1)readfiforam<=1;//������ �������� ������ fiforam
					else readfiforam<=0; end
// ��������� �������� getfiforam �� ���������� ������� � ����� ����� ���������
always @(posedge readfiforam or posedge resgetfiforam)
			if(resgetfiforam==1)getfiforam<=0;
				else getfiforam<=1;
// ������ ������ �� fifo �� ARM
always @(posedge bls0we or posedge getfifo)
	if(getfifo==1)readfifo<=0;
		else begin if(adrRKS==1 && cs==0 && databus[4]==1 ) readfifo<=1;// ������ �������� ������ fifo
					else readfifo<=0; end
//��������� �������� getfifo �� ���������� ������� � ����� ����� ���������
always @(posedge readfifo or posedge resgetfifo)
			if(resgetfifo==1)getfifo<=0;
				else getfifo<=1;//��������� ������ �� fifotimer
/*
// ������ ������� ����� ������ ������ 1 � RKS[0]
always @(posedge bls0we or posedge resetgo)
	begin
	if(resetgo==1)begin go<=0; end // ��������� ���������� ����� ����� �������� � ������� ������������
	else begin	if(adrRKS==1 && cs==0 && databus[0]==1 ) begin go<=1; end// ������ � ������� ���������-����� �������� �������
		 end
	end
 */
//������ ������� �������� ��� slave �������
always @(posedge rksgo or posedge resetgo)
	begin
	if(resetgo==1)begin go<=0; end // ��������� ���������� ����� ����� �������� � ������� ������������
	else begin	/*if(rksgo==1 )*/ begin go<=1; end// ������ � ������� ���������-����� �������� �������
		 end
	end


// ��������� ����������� ����� �������� � �������� TIMER � ������������� ��� ����� ��������

always @(posedge go or posedge fixtimer)
	if(fixtimer==1)firsttimer<=0;
		else firsttimer<=1;
//
always @(*)
	// ��������� ��� ������ ������� 
	begin
	mreset<=~0;//��������� �������� �� 0 - �� ������� ������ 0 (BWL) ������ ����� LOW BW
	mfield<=~0;//��������� �������� �� 0- �� ������� ������ 0 (GAIN)���� ������� � ������� ��� ���������� ������ ������� ���� ��������
	end
//���������� ������ ���������
always @(*)
begin
	if(adresbus==6'b000000)adrRKS<=1;		// RKS		0 �����
		else	adrRKS<=0;
	if(adresbus==6'b000001)adrRKSMEM<=1;	// RKSMEM	1 �����
		else	adrRKSMEM<=0;
	if(adresbus==6'b000010)adrDATA<=1;		// DATA		2 �����
		else	adrDATA<=0;
	if(adresbus==6'b000100)adrTIMER0<=1;	// TIMER0	4 �����
		else	adrTIMER0<=0;
	if(adresbus==6'b000101)adrTIMER1<=1;	// TIMER1	5 �����
		else	adrTIMER1<=0;
	if(adresbus==6'b000110)adrTIMER2<=1;	// TIMER2	6 �����
		else	adrTIMER2<=0;
	if(adresbus==6'b000111)adrTIMER3<=1;	// TIMER3	7 �����
		else	adrTIMER3<=0;
	if(adresbus==6'b001000)adrPOROG0<=1;	// POROG0	8 �����
		else	adrPOROG0<=0;
	if(adresbus==6'b001001)adrPOROG1<=1;	// POROG1	9 �����
		else	adrPOROG1<=0;
	if(adresbus==6'b001010)adrFIFO0<=1;		// FIFOTIM0	10 �����
		else	adrFIFO0<=0;
	if(adresbus==6'b001011)adrFIFO1<=1;		// FIFOTIM1	11 �����
		else	adrFIFO1<=0;
	if(adresbus==6'b001100)adrFIFO2<=1;		// FIFOTIM2	12 �����
		else	adrFIFO2<=0;
	if(adresbus==6'b001101)adrFIFO3<=1;		// FIFOTIM3	13 �����
		else	adrFIFO3<=0;
	if(adresbus==6'b001110)adrRAM0<=1;		// FIFORAM0	14 �����
		else	adrRAM0<=0;
	if(adresbus==6'b001111)adrRAM1<=1;		// FIFORAM1	15 �����
		else	adrRAM1<=0;
	if(adresbus==6'b010000)adrRAM2<=1;		// FIFORAM3	16 �����
		else	adrRAM2<=0;
	if(adresbus==6'b010001)adrRAM3<=1;		// FIFORAM4	17 �����
		else	adrRAM3<=0;
	if(adresbus==6'b010010)adrRKSRAM<=1;	// RKSRAM	18 �����
		else	adrRKSRAM<=0;

end
//������ � ��������
always @(posedge bls0we)
begin
	if((adrRKS==1) && (cs==0))RKS<=databus;
	if((adrRKSMEM==1) && (cs==0))RKSMEM<=databus;
	if((adrPOROG0==1) && (cs==0))POROG[7:0]<=databus;
	if((adrPOROG1==1) && (cs==0))POROG[9:8]<=databus[1:0];
end
//������ �� ���������
always @(oe)
begin
	if(oe==0)	
		begin
		case ({adrRKSRAM,adrRAM3,adrRAM2,adrRAM1,adrRAM0,adrFIFO3,adrFIFO2,adrFIFO1,adrFIFO0,adrTIMER3,adrTIMER2,adrTIMER1,adrTIMER0,adrDATA,adrPOROG1,adrPOROG0,adrRKSMEM,adrRKS,cs})
		19'b0000000000000000010			:databus<={~go,intakadr|intckadr,getfifo,1'b0,timerempty,timerfull,vistrel,RKS[0]};
// �������� ���� ���������������� �������
//		19'b0000000000000000010			:databus<={~go,intakadr,getfifo,1'b0,timerempty,timerfull,vistrel,RKS[0]};
		19'b0000000000000000100			:databus<=RKSMEM;
		19'b0000000000000001000			:databus<=POROG[7:0];
		19'b0000000000000010000			:databus<={6'b000000,POROG[9:8]};
		19'b0000000000000100000			:databus<=datamem; // ������ �� RAM ������� ���������� ������ �� ������ ������������� ������ � ������ ������
		19'b0000000000001000000			:databus<=REGTIMER[7:0]; //������ ��������
		19'b0000000000010000000			:databus<=REGTIMER[15:8]; //������ ��������
		19'b0000000000100000000			:databus<=REGTIMER[23:16]; //������ ��������
		19'b0000000001000000000			:databus<={1'b0,REGTIMER[30:24]}; //������ ��������
		19'b0000000010000000000			:databus<=datafifo[7:0]; //fifo ������ ��������
		19'b0000000100000000000			:databus<=datafifo[15:8]; //fifo ������ ��������
		19'b0000001000000000000			:databus<=datafifo[23:16]; //fifo ������ ��������
		19'b0000010000000000000			:databus<={1'b0,datafifo[30:24]}; //fifo ������ ��������
		19'b0000100000000000000			:databus<=ramfifo[7:0];
		19'b0001000000000000000			:databus<={7'b0000000,ramfifo[8]};
		19'b0010000000000000000			:databus<=ramfifo[16:9];
		19'b0100000000000000000			:databus<={7'b0000000,ramfifo[17]};
		19'b1000000000000000000			:databus<={2'b00,getfiforam,1'b0,ramempty,ramfull,2'b00};
		default			:databus<=8'bzzzzzzzz;
		endcase
		end
	else begin databus<=8'bzzzzzzzz; end
end
//������ � ��� ����� ����
//������������ ������ ����� ADC � CPU		
//����� �� ADC ��� �� CPU ��������
always	@(RKSMEM[0])
if (RKSMEM[0]==1)adresmem<=adrregADC;
		else	adresmem<=adrreg;
// ������� �� ������ � ������ � ������
// ���� ����� � ������ �� ���� ���������� ���� � oemem=bls0we
// ���� �� ����� ���� �� ������ ������ � �� ���� ���������� ����� ������
always@(*)
	begin
// ���� ������ ����������� ARM
	if((RKSMEM[0]==0) && (adrDATA==1) && (cs==0) && (bls0we==0)) begin datamem<=databus; end //���� � ������ ��� ������
		else begin datamem<=8'bzzzzzzzz; end // ������ ��� ����� � ��������� ������
	if((adrDATA==1) && (cs==0) && (bls0we==0)) begin wememARM<=0; oememARM<=1; end //���� � ������ ��� ������
		else begin wememARM<=1; oememARM<=0; end // ������ ��� ����� � ��������� ������
// ���� ������ ����������� ADC
//������������� �������� ��� ������ � ����� �������
	if(RKSMEM[0]==0) begin wemem<=wememARM; oemem<=oememARM;	end
		else begin wemem<=wememADC; oemem<=1; datamem<=~adcreg[11:4];end // ��� ����� � ������ � ������ � ADC ����
	end
	
//������ � adrreg ����������������� �� ������ ������ � ������
always @(posedge RKSMEM[7] or posedge a192mhz)
	begin
	if(RKSMEM[7]==1)begin adrreg<=0; wait192<=0; resadd<=0; end // ���������� adrreg �� 1 � RKSMEM[7]
		else begin  if(read==1 || write==1) wait192<=wait192+1; // ������� � ����� a192mhz
						else begin wait192<=0; resadd<=0; end
					if(wait192==2)begin adrreg=adrreg+1; resadd<=1;end // inc ����� � ���������� ������� inc
			 end
	end

always @(posedge oe or posedge resadd)
	if(resadd==1)read<=0;
		else if(adrDATA==1 && cs==0 && RKSMEM[1]==1) read<=1; //���������� �������  inc �� read
		
always @(posedge bls0we or posedge resadd)
	if(resadd==1)write<=0;
		else if(adrDATA==1 && cs==0 && RKSMEM[2]==1) write<=1; // ���������� ������� inc �� write

assign svetodiod=gostart; // ������ ����������� 
//assign rksgo=RKS[0]; // ������ ������� ���� ������� ��� master ���� �� RKS[0] ��� slave �� ������ �������
 assign rksgo=ingo; // ��� slave �������  � ������� outgo pin ���������� � slave ingo 


endmodule
