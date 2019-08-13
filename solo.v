module solo(a12mhz,svetodiod,adresbus,databus,cs,oe,bls0we,adresmem,datamem,oemem,wemem,
			mreset,mfield,mdata,mfsync,mlsync,mclock,clockADC,
			acdata,foto,ingo,ready,go);
parameter lineclk=320/2; //число пикселов в строке/2 число clk в строке - правильное число импульсов в строке
parameter line=256;//число линий в матрице - с этим работало
parameter psw=33'b010000010010000000000000000000010; // слово состояния для сброса матрицы
parameter block=300/10;// время блокирования xxx/10 ns повторного выстрела для записи в FIFO xxx должно быть больше 3 !!!
//parameter dlinay=261;// a?aiy eioaa?e?iaaiey i?e aeeiiie auaa??ea aac auno?aea 261-118+50 =
//parameter dlinaImit=361;
//parameter dlin=183;// iauee ei?ioeee ia?eia eioaa?e?iaaiea 
parameter dlinay=10000;// a?aiy eioaa?e?iaaiey i?e aeeiiie auaa??ea aac auno?aea 261-118+50 =
parameter dlinaImit=10000;// aey aeeiiie auaa??ee aey eaeea?iaee
parameter dlin=25000;// iauee ei?ioeee ia?eia eioaa?e?iaaiea 
//сигналы на матрицу 
output mreset,mfield,mdata,mfsync,mlsync,mclock;
reg mreset,mfield;
wire mclock,mfsync,mlsync,mdata;
//сигналы с ADC и на  ADC
output	clockADC;
reg clockADC;
input	[12:0]	acdata;
reg [12:0] adcreg;
// сигнал с фотодиода
input	foto;
//переменные основной программы
input a12mhz;	//входная частота для PLL
output svetodiod; // мигает светодиодик
wire svetodiod;
wire a96mhz;	//выходная частота PLL - основная тактовая частота
wire a192mhz;	// вторая выходная частота
wire plllocked;	// частота установилась
reg [13:0] div9600; // 100 us импульсы
reg [30:0] timer; // счетчик времени
reg sttimer; // сброс и разрешение работы таймера управляется RKS[6]
wire [30:0] datafifo;//данные из fifo таймера
wire timerempty,timerfull; //fifo таймера пустое и заполненное
reg [30:0] REGTIMER; // регистр момента выстрела
wire autovistrel;// суммарный сигнал от foto и arm имитации
reg vistrel; // триггер произошел выстрел на время блокировки сигнала
reg stvistrel;
reg kadrvistrel; // признак выстрела для кадра
reg kadrvi;// признак выстрела в clock домене 96MHz
reg intakadr,intbkadr,intckadr; //если пришел выстрел и мы интегрируем свет в зависимости от фазы
reg intckadrsecond; // пошел второй кадр после пришедшего выстрела в фазе intckadr
reg intbkadrsecond; // пошел второй кадр после выстрела с удлиненной выдержкой
reg intbkadrthree;// третий кадр после выстрела
output go; //матрица выполняет действия
reg go; // сигнал запуска матрицы - после запуска работы матрицы =1
input ingo; // входной сигнал синхронизации
wire ingo;
wire rksgo ; //замена сигнала RKS[0] в другую матрицу на U37 73 pin -> на U39 4 pin
reg firstslave;// однократность сигнала запуска из мастера в slave
reg gostart; // сигнал go переведенный в clock домен
reg resetgo; // сброс сигнала go разрешающего циклы матриц до момента считывания кадра
reg waitgo; //переход в клоковый домен
reg imit; //триггер имитации выстрела устанавливается от ARM
reg [18:0] dlinatime; // регистр определяющий фон второго кадра 
reg resvistrel; // конец обработки vistrel
reg [15:0] waittick;
reg waitkadr;
reg fixstartkadr; // зафиксировали выстрел в период интегрирования
reg firsttimer;// первый выстрел с обработкой матриц
reg fixtimer;// время зафиксировано и будет изменено только на следующем запуске матриц
reg [3:0] fifowait;// синхронизатор с доменом a96MHz
reg readfifo; // запрос из arm для чтения fifo
reg getfifo;// триггер произошел запрос на чтение fifo
reg stgetfifo;// переход в клоковый домен
reg resgetfifo;// окончание сброс обработки чтения из fifo
reg wrfifo;// разрешение записи в fifo timer
reg rdfifo;// разрешение чтения из fifo timer
wire flash,flasha; //переменные логики выстрела для кадра
output ready;// сигнал на разрешенеи приема кадра в первую матрицу
reg ready;// разрешение фиксирования выстрела внутри запущенной процедуры кадра

//перменные для работы с координатами из fifo выше порога
reg wrram;// запись в fifo то что выше порога cndata и linecount
reg rdram; // чтение из fifo координат выше порога
wire ramempty; //fifo пусто
wire ramfull; // fifo переполнено и надо обрабатывать калры из памяти и при этом уменьшить усиление
wire [17:0] ramfifo;// данные из fifo наружу
reg getfiforam,readfiforam,resgetfiforam,stgetfiforam,fifowaitram;

//переменные автомата работы матрицы и ацп
reg [3:0] faza; //фаза обеспечивает деление a96mhz
reg clocka;// clock который нужно подать на матрицу
reg [18:0] kodclocka; // номер clock который сейчас отрабатывается матрицей(атоматом состояния)
reg mfclocka;//вывод на матрицу сигнала frame
reg mfsynca,mlsynca; 
reg [32:0] sdvigpsw;
reg [32:0] sdvig; // регистр смешещения при переходе к следующей строке
reg [10:0] cndata;// счетчик состояний clock в строке -номер пиксела
reg [8:0] linecount; // номер строки кадра
reg [10:0] cnreg;
reg [8:0] linereg;
reg testimit;// сигнал выстрела определнной фазы для отладки

// переменные модуля памяти
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

reg wememARM,oememARM;// сигналы записи чтения на память от ARM
reg wememADC; // сигнал записи на пять от ADC oemem=1 постоянно
reg [18:0]	adrreg; // регистр адреса на память когда работаем от ARM
reg [2:0]	wait192;
reg read,write,resadd;
reg [18:0]	adrregADC; // регистр адреса на память когда работаем от ADC
reg [30:0] temptimer; // временная переменная

//end переменных модуля памяти
// переменные выбора регистра
reg adrRKS,adrRKSMEM,adrDATA,adrTIMER0,adrTIMER1,adrTIMER2,adrTIMER3,adrFIFO0,adrFIFO1,adrFIFO2,adrFIFO3;
reg adrPOROG0,adrPOROG1;
reg adrRAM0,adrRAM1,adrRAM2,adrRAM3; //регистры fiforam пикселов больше POROG
reg adrRKSRAM;
reg [7:0] RKS;
reg [7:0] RKSMEM;
reg [9:0] POROG;

// пошла программа
pllgen mypll(.inclk0(a12mhz),.c0(a96mhz),.c1(a192mhz),.locked(plllocked)); // PLL для умножения опорной частоты в основную тактовую
fifotimer myfifotimer(.clock(a96mhz),.data(temptimer),.rdreq(rdfifo),.wrreq(wrfifo),.empty(timerempty),.full(timerfull),.q(datafifo)); //fifo для записи timer выстрелов
myfiforam fiforam(.clock(a96mhz),.data({cnreg[8:0],linereg[8:0]}),.rdreq(rdram),.wrreq(wrram),.empty(ramempty),.full(ramfull),.q(ramfifo)); // fifo для записи пикселов выше порога
//автомат чтения и записи в FIFO и фиксация времени выстрела в регистре и фифо
always @(posedge a96mhz)
	begin
	if(RKS[6]==1)sttimer<=1; //переходим в клоковый домен
	else sttimer<=0;
	if (sttimer==1)begin timer<=0;div9600<=0; temptimer<=0; end //RKS[6]==1 удерживает таймер в 0. запись 0 приводит к счету таймера
		else begin if(div9600==9599)begin
					 div9600<=0;
					 if(timer==863999999)begin timer<=0; end
						else begin timer<=timer+1; temptimer<=timer; end
					end
		else begin div9600<=div9600+1; end
			end
// записывем время выстрела матрицы
	if(kadrvistrel==1)waitkadr<=1;
		else waitkadr<=0;
	if(waitkadr==1 && firsttimer==1)begin REGTIMER<=temptimer; fixtimer<=1; end // таймер зафиксировали и блокируем до запуска новой операции
		else fixtimer<=0;

	if(vistrel==1)stvistrel<=1;// переходим в клоковый  домен a96MHz
	if(stvistrel==1)waittick<=waittick+1;//пропускаем 1 clock 96MHz для перехода в домен и block clock для блокировки дребезга от фотодиода
		else waittick<=0;
	if(waittick==block)begin resvistrel<=1;stvistrel<=0; end //(время блокировки фотодиода) выстрел обработан и сбрасываем vistrel<=0 путем resvistrel<=1 
				else resvistrel<=0;
// запись в fifo событий выстрела
	if(waittick==1 )wrfifo<=1; // записываем данные timer в fifo
		else wrfifo<=0;
// чтение данных из fifotimer
	if(getfifo==1)stgetfifo<=1;// переходим в клоковый домен a96MHZ - можно не переходить так как счетчик fifowait 1 бит !!!
	if(stgetfifo==1)fifowait<=fifowait+1;
		else fifowait<=0;
	if(fifowait==1)begin rdfifo<=1; resgetfifo<=1; stgetfifo<=0;end // чтение из fifo
		else begin rdfifo<=0; resgetfifo<=0; end
// чтение fiforam  по аналогии с fifotimer

	if(getfiforam==1)stgetfiforam<=1;// переходим в клоковый домен a96MHZ - можно не переходить так как счетчик fifowait 1 бит !!!
	if(stgetfiforam==1)fifowaitram<=fifowaitram+1;
		else fifowaitram<=0;
	if(fifowaitram==1)begin rdram<=1; resgetfiforam<=1; stgetfiforam<=0;end // чтение из fifo
		else begin rdram<=0; resgetfiforam<=0; end

	end
//автомат работы матрицы
// сигналы на матрицу
assign mclock=~clocka; //clock на матрицу на матрицу идет через инвертор
assign mfsync=~mfsynca;//на матрицу подается через инвертор вне ПЛИС
assign mlsync=~mlsynca;//на матрицу подается через инвертор
assign mdata=~sdvigpsw[32]; // выдаем данный наружу в инверсии через внешний инвертор

always @(posedge a96mhz)
	begin
	if(kadrvistrel==1)kadrvi<=1;//переходим в clock домен выстрел пришел
		else kadrvi<=0;
	if(go==1 && rksgo==0)waitgo<=1; //переходим в клоковый домен после того как передернули RKS[0] =0->1->0
		else  begin waitgo<=0; gostart<=0; end
	if (waitgo==1)begin gostart<=1; end 
	if(imit==1 && kodclocka==80 )begin testimit<=1; dlinatime<=dlinaImit; end//имитация выстрела в фазе B - самая большая выдержка
		else testimit<=0; 
	if(rksgo==1)begin fixstartkadr<=0; intakadr<=0; intbkadr<=0; intckadr<=0; intckadrsecond<=0; 
					intbkadrsecond<=0; intbkadrthree<=0; adrregADC<=0; linecount<=0;sdvig<=0; cndata<=0; end //сброс по записи 1 в регистр RKS[0]
	if(gostart==1)	begin //если разрешено работать матрице go==1
		if(faza==9)faza<=0;// деление частоты на 10 
			else faza<=faza+1; // faza внутри полупериода clock faza==0 соответсвует изменению clock
// циклограмма записи в видео память если мы уже на 3 строке и после 3 пиксела+3 цикла ADC
//                        внутри строки с учетом сдвига ADC        внутри  2 до 242 строк
				if(cndata>=11 && cndata<=329 && linecount>=4 && linecount<=258 && faza>=7 && faza<=9)wememADC<=0; //формируем we в память
					else wememADC<=1;
// запись в FIFORAM одновременно с  записью в память если значение POROG[9:0]<=acdata[11:2]
				if(cnreg>=11 && cnreg<=329 && linereg>=4 && linereg<=258 && faza==9
				 && (POROG<=~adcreg[11:2]))wrram<=1; //формируем we в память
					else wrram<=0;
					
					// в начале записи элемента увеличиваем адрес на память получается что нулевой элемент лишний
				if(cndata>=11 && cndata<=329 && linecount>=4 && linecount<=258 && faza==2)adrregADC<=adrregADC+1; 
// clockADC сдвинут относительно clock матрицы на 2 такта=20 нс
				if(faza>=2 &&  faza<=6) clockADC<=1;// формируем clock ADC
					else clockADC<=0;
				if(faza==6)begin adcreg=acdata; cnreg<=cndata; linereg<=linecount; end
//отрабатывается циклограмма матрицы все изменения делаются в faza=0
		if(faza==0 )begin
				//  без выстрела                         
				if(((kodclocka>=dlin) && (intakadr==0 && intbkadr==0 && intckadrsecond==0)) || 
					((kodclocka>=dlinatime)&&(intbkadr==1)&&(intbkadrthree==0))) // удлиненная выдержка для считывания фона в кадре без выстрела
							begin
							kodclocka<=0;//если выстрела нет то короткий повтор
							clocka<=0;
							// сделали пустой кадр и разрешили обрабатывать выстрелы
							ready<=1;// вошли в процедуру кадрирования -> можно разрешить прием выстрела
							if(intckadr==1)intckadrsecond<=1; //блокировка для перехода в считывание второго кадра в фазе C
							if(intbkadr==1)intbkadrsecond<=1; //блокировка для перехода в считывание второго кадра в фазе B
							if(intbkadrsecond==1)intbkadrthree<=1;// блокировка второго кадра и чтение только 3 кадра в фазе B
							end
					else begin kodclocka<=kodclocka+1; clocka<=clocka+1;  end//изменяем состояние автомата в начале фазы
				if(kodclocka>=0 && kodclocka<=117)mfsynca<=1; //формирование MFSYNC длиной 59 clocks по переднему фронту clock
					else mfsynca<=0;
// формирование строчных импульсов внутри кадра
				if(kodclocka<=(line+2)*(lineclk+16)*2 && kodclocka>=(1+sdvig) && kodclocka<=(2+sdvig))mlsynca<=1; // формируем mlsynca
					else mlsynca<=0;
				if((kodclocka >=52) && (intakadr==0) && (intckadrsecond==0) && (intbkadrthree==0)) //26 clocks от MFSYNC и делаем reset
					begin
					if(kodclocka[0]==0)begin sdvigpsw[32:1]<=sdvigpsw[31:0];sdvigpsw[0]<=0; end
					end
					else sdvigpsw=psw;// делаем ресет		
// если выстрел то считываем кадр
// считаем строки и индексы начала новой строки в clock
		//пошел кадр

				if(kodclocka==(3+sdvig))linecount<=linecount+1;// строки с 1 по ..
				if(cndata>=((lineclk+16)*2-1))begin cndata<=0; end  // считаем номер строки при конце строки
					else cndata<=cndata+1;// считаем номер пиксела в строке с 0 по ..
				if(cndata==(lineclk*2+12))sdvig<=sdvig+(lineclk+16)*2; //в конце строки определяем новые индексы kodclocka
			end
	end
//фазы прихода выстрела и состояния матрицы интегрирования
//     первые 25 clock                         от заднего фронта fsync
	if((0<=kodclocka && kodclocka<=50) && (kadrvi==1) && fixstartkadr==0)
/*		begin intakadr<=1; end //выстрел попал в фазу до начала ресета можно сразу читать кадр - для мастер матрицы*/
		begin intbkadr<=1; end //для slave в фазе A идет фаза C в мастере - поэтому делаем фазу B
//фаза B - матрица находится в стадии сброса и получаем темновой кадр - без выстрела
	if((51<=kodclocka && kodclocka<=116) && (kadrvi==1) && fixstartkadr==0)
		begin intbkadr<=1; end //выстрел попал в период сброса - нужно пропустить кадр и сделать еще пустые тики чтобы итегрирование было больше
	if((117<=kodclocka && kodclocka<=dlin) && (kadrvi==1) && fixstartkadr==0)
		begin intckadr<=1; end //выстрел попал в интегрирование и после сброса просто можно начать считывание кадра
	if(kadrvi==1) fixstartkadr<=1; // как только пришел выстрел фиксируем было ли интегрирование или нет!!
// закончили читать кадр делаем запрет на повторный цикл
	if(kodclocka==((line+2)*(lineclk+16)*2+200+1))begin resetgo<=1; dlinatime<=dlinay; ready<=0; end //считали кадр и остановились до следующего запуска
		else resetgo<=0;
//если считали кадр то делаем начальное состояние	
	if(resetgo==1)begin
			kodclocka<=0;
			clocka<=0;
			end
	if(kodclocka==0) // если мы в начале кадра то делаем установку всех переменных
		begin
		sdvig<=0;cndata<=0;linecount<=0;adrregADC<=0;
		end
	// если выстрел то reset не надо делать
// считывание кадра в память - формируются сигналы для считывания всей матрицы
// определяем что длина строки равна (lineclk+16)*2   делаем 16 пустых clock в конце строки
// а длина кадра равна (line+2)*(lineclk+16)*2+200 делаем 100 пустых clock в конце кадра
end
//имитация выстрела из ARM
//сигнал либо от фотодиода либо от ARM запись 1 в -> RKS[1]
always @(posedge bls0we or posedge resvistrel)
		if(resvistrel==1)imit<=0;
		else if(adrRKS==1 && cs==0 && databus[1]==1)imit<=1;
//or (autovistrel,foto,imit);//общее место от выстрела и от имитации
or (autovistrel,foto,testimit);// для калибровки имитация выстрела делается в фазе B самая большая выдержка
or (flasha,foto,testimit); // отдельно вспышка для кадра разрешена только когда ready
and (flash,flasha,ready);// вспышка только внутри цикла кадров ready==1 если запущен цикл кадров
// для кадров
//по запуску системы скидывается признак и по приходу выстрела признак устанавливается и стоит пока по новой не перезапустят
always @(posedge flash or posedge rksgo) // сигнал для работы матрицы с выстрелом
	if(rksgo==1 )kadrvistrel<=0;  
		else kadrvistrel<=1;
//для таймера
//установка триггера vistrel по пришедшему сигналу и сброс после обработки
always @(posedge autovistrel or posedge resvistrel)
			if(resvistrel==1)vistrel<=0;
				else vistrel<=1;//произошел выстрел и надо фиксировать
// запрос чтения из fifoRAM из ARM делаем запись 4 бит и устанавлвается getfiforam которое будет сброшено resgetfiforam
always @(posedge bls0we or posedge getfiforam)
	if(getfiforam==1)readfiforam<=0;
		else begin if(adrRKSRAM==1 && cs==0 && databus[4]==1)readfiforam<=1;//сигнал признака чтения fiforam
					else readfiforam<=0; end
// установка триггера getfiforam по пришедшему сигналу и сброс после обработки
always @(posedge readfiforam or posedge resgetfiforam)
			if(resgetfiforam==1)getfiforam<=0;
				else getfiforam<=1;
// запрос чтения из fifo из ARM
always @(posedge bls0we or posedge getfifo)
	if(getfifo==1)readfifo<=0;
		else begin if(adrRKS==1 && cs==0 && databus[4]==1 ) readfifo<=1;// сигнал признака чтения fifo
					else readfifo<=0; end
//установка триггера getfifo по пришедшему сигналу и сброс после обработки
always @(posedge readfifo or posedge resgetfifo)
			if(resgetfifo==1)getfifo<=0;
				else getfifo<=1;//запустили чтение из fifotimer
/*
// сигнал запуска цикла матриц запись 1 в RKS[0]
always @(posedge bls0we or posedge resetgo)
	begin
	if(resetgo==1)begin go<=0; end // произошло считывания кадра после выстрела и матрицы остановились
	else begin	if(adrRKS==1 && cs==0 && databus[0]==1 ) begin go<=1; end// сигнал о запуске процедуры-пошла работать матрица
		 end
	end
 */
//сигнал запуска операции для slave матрицы
always @(posedge rksgo or posedge resetgo)
	begin
	if(resetgo==1)begin go<=0; end // произошло считывания кадра после выстрела и матрицы остановились
	else begin	/*if(rksgo==1 )*/ begin go<=1; end// сигнал о запуске процедуры-пошла работать матрица
		 end
	end


// разрешаем фиксировать время выстрела в регистре TIMER и заблокировать его после фиксации

always @(posedge go or posedge fixtimer)
	if(fixtimer==1)firsttimer<=0;
		else firsttimer<=1;
//
always @(*)
	// параметры для другой матрицы 
	begin
	mreset<=~0;//поскольку инверсия то 0 - на матрицу придет 0 (BWL) жестко равно LOW BW
	mfield<=~0;//поскольку инверсия то 0- на матрицу придет 0 (GAIN)надо добавит в регистр для управления сейчас большой коэф усиления
	end
//дешифратор адреса регистров
always @(*)
begin
	if(adresbus==6'b000000)adrRKS<=1;		// RKS		0 адрес
		else	adrRKS<=0;
	if(adresbus==6'b000001)adrRKSMEM<=1;	// RKSMEM	1 адрес
		else	adrRKSMEM<=0;
	if(adresbus==6'b000010)adrDATA<=1;		// DATA		2 адрес
		else	adrDATA<=0;
	if(adresbus==6'b000100)adrTIMER0<=1;	// TIMER0	4 адрес
		else	adrTIMER0<=0;
	if(adresbus==6'b000101)adrTIMER1<=1;	// TIMER1	5 адрес
		else	adrTIMER1<=0;
	if(adresbus==6'b000110)adrTIMER2<=1;	// TIMER2	6 адрес
		else	adrTIMER2<=0;
	if(adresbus==6'b000111)adrTIMER3<=1;	// TIMER3	7 адрес
		else	adrTIMER3<=0;
	if(adresbus==6'b001000)adrPOROG0<=1;	// POROG0	8 адрес
		else	adrPOROG0<=0;
	if(adresbus==6'b001001)adrPOROG1<=1;	// POROG1	9 адрес
		else	adrPOROG1<=0;
	if(adresbus==6'b001010)adrFIFO0<=1;		// FIFOTIM0	10 адрес
		else	adrFIFO0<=0;
	if(adresbus==6'b001011)adrFIFO1<=1;		// FIFOTIM1	11 адрес
		else	adrFIFO1<=0;
	if(adresbus==6'b001100)adrFIFO2<=1;		// FIFOTIM2	12 адрес
		else	adrFIFO2<=0;
	if(adresbus==6'b001101)adrFIFO3<=1;		// FIFOTIM3	13 адрес
		else	adrFIFO3<=0;
	if(adresbus==6'b001110)adrRAM0<=1;		// FIFORAM0	14 адрес
		else	adrRAM0<=0;
	if(adresbus==6'b001111)adrRAM1<=1;		// FIFORAM1	15 адрес
		else	adrRAM1<=0;
	if(adresbus==6'b010000)adrRAM2<=1;		// FIFORAM3	16 адрес
		else	adrRAM2<=0;
	if(adresbus==6'b010001)adrRAM3<=1;		// FIFORAM4	17 адрес
		else	adrRAM3<=0;
	if(adresbus==6'b010010)adrRKSRAM<=1;	// RKSRAM	18 адрес
		else	adrRKSRAM<=0;

end
//запись в регистры
always @(posedge bls0we)
begin
	if((adrRKS==1) && (cs==0))RKS<=databus;
	if((adrRKSMEM==1) && (cs==0))RKSMEM<=databus;
	if((adrPOROG0==1) && (cs==0))POROG[7:0]<=databus;
	if((adrPOROG1==1) && (cs==0))POROG[9:8]<=databus[1:0];
end
//чтение из регистров
always @(oe)
begin
	if(oe==0)	
		begin
		case ({adrRKSRAM,adrRAM3,adrRAM2,adrRAM1,adrRAM0,adrFIFO3,adrFIFO2,adrFIFO1,adrFIFO0,adrTIMER3,adrTIMER2,adrTIMER1,adrTIMER0,adrDATA,adrPOROG1,adrPOROG0,adrRKSMEM,adrRKS,cs})
		19'b0000000000000000010			:databus<={~go,intakadr|intckadr,getfifo,1'b0,timerempty,timerfull,vistrel,RKS[0]};
// проверял фазы чувствительности матрицы
//		19'b0000000000000000010			:databus<={~go,intakadr,getfifo,1'b0,timerempty,timerfull,vistrel,RKS[0]};
		19'b0000000000000000100			:databus<=RKSMEM;
		19'b0000000000000001000			:databus<=POROG[7:0];
		19'b0000000000000010000			:databus<={6'b000000,POROG[9:8]};
		19'b0000000000000100000			:databus<=datamem; // чтение из RAM сигналы управления всегда на чтение переключаются только в момент записи
		19'b0000000000001000000			:databus<=REGTIMER[7:0]; //таймер выстрела
		19'b0000000000010000000			:databus<=REGTIMER[15:8]; //таймер выстрела
		19'b0000000000100000000			:databus<=REGTIMER[23:16]; //таймер выстрела
		19'b0000000001000000000			:databus<={1'b0,REGTIMER[30:24]}; //таймер выстрела
		19'b0000000010000000000			:databus<=datafifo[7:0]; //fifo таймер выстрела
		19'b0000000100000000000			:databus<=datafifo[15:8]; //fifo таймер выстрела
		19'b0000001000000000000			:databus<=datafifo[23:16]; //fifo таймер выстрела
		19'b0000010000000000000			:databus<={1'b0,datafifo[30:24]}; //fifo таймер выстрела
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
//работа с ОЗУ через ПЛИС
//переключение памяти между ADC и CPU		
//адрес от ADC или от CPU регистра
always	@(RKSMEM[0])
if (RKSMEM[0]==1)adresmem<=adrregADC;
		else	adresmem<=adrreg;
// сигналы на чтение и запись в память
// если пишем в память то шину направляем туда и oemem=bls0we
// если не пишем туда то читаем всегда и на шину выставляем когда читаем
always@(*)
	begin
// если память принадлежит ARM
	if((RKSMEM[0]==0) && (adrDATA==1) && (cs==0) && (bls0we==0)) begin datamem<=databus; end //шина в память при записи
		else begin datamem<=8'bzzzzzzzz; end // память все время в состоянии чтения
	if((adrDATA==1) && (cs==0) && (bls0we==0)) begin wememARM<=0; oememARM<=1; end //шина в память при записи
		else begin wememARM<=1; oememARM<=0; end // память все время в состоянии чтения
// если память принадлежит ADC
//мультиплексор сигналов при работе с видео памятью
	if(RKSMEM[0]==0) begin wemem<=wememARM; oemem<=oememARM;	end
		else begin wemem<=wememADC; oemem<=1; datamem<=~adcreg[11:4];end // все время в записи и данные с ADC туда
	end
	
//работа с adrreg автоинкрементация по чтению записи в память
always @(posedge RKSMEM[7] or posedge a192mhz)
	begin
	if(RKSMEM[7]==1)begin adrreg<=0; wait192<=0; resadd<=0; end // сбрасываем adrreg по 1 в RKSMEM[7]
		else begin  if(read==1 || write==1) wait192<=wait192+1; // переход в домен a192mhz
						else begin wait192<=0; resadd<=0; end
					if(wait192==2)begin adrreg=adrreg+1; resadd<=1;end // inc адрес и сбрасываем признак inc
			 end
	end

always @(posedge oe or posedge resadd)
	if(resadd==1)read<=0;
		else if(adrDATA==1 && cs==0 && RKSMEM[1]==1) read<=1; //выставляем признак  inc по read
		
always @(posedge bls0we or posedge resadd)
	if(resadd==1)write<=0;
		else if(adrDATA==1 && cs==0 && RKSMEM[2]==1) write<=1; // выставляем признак inc по write

assign svetodiod=gostart; // мигаем светодиодом 
//assign rksgo=RKS[0]; // сигнал запуска кдра матрицы для master идет из RKS[0] для slave из другой матрицы
 assign rksgo=ingo; // для slave матрицы  в мастере outgo pin передается в slave ingo 


endmodule
