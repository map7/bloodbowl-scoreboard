// 8 Digit 7 segment display

#define MAX7219DIN 4
#define MAX7219CS 5
#define MAX7219CLK 6

void setup() {
  MAX7219init();
  MAX7219brightness(15);
}

void loop() {
//  for(byte i=0;i<8;i++){
//    MAX7219senddata(i+1,i);
//  }

  for(byte i=0;i<8;i++){
    MAX7219senddata(i+1,15);
  }

  // Display 3:00
  //MAX7219senddata(4,10); // minus
  //MAX7219senddata(4,11); // E
  //MAX7219senddata(4,12); // H
  //MAX7219senddata(4,13); // L
  //MAX7219senddata(4,14); // P
  //MAX7219senddata(4,239);  // .
  //MAX7219senddata(4,240);  // 0.
  //MAX7219senddata(4,241);  // 1.


  MAX7219senddata(3,243);
  
  MAX7219senddata(2,0);
  MAX7219senddata(1,0);
  delay(1000);
}

void MAX7219brightness(byte b){  //0-15 is range high nybble is ignored
  MAX7219senddata(10,b);        //intensity  
}

void MAX7219init(){
  pinMode(MAX7219DIN,OUTPUT);
  pinMode(MAX7219CS,OUTPUT);
  pinMode(MAX7219CLK,OUTPUT);
  digitalWrite(MAX7219CS,HIGH);   //CS off
  digitalWrite(MAX7219CLK,LOW);   //CLK low
  MAX7219senddata(15,0);        //test mode off
  MAX7219senddata(12,1);        //display on
  MAX7219senddata(9,255);       //decode all digits
  MAX7219senddata(11,7);        //scan all
  for(int i=1;i<9;i++){
    MAX7219senddata(i,0);       //blank all
  }
}

void MAX7219senddata(byte reg, byte data){
  digitalWrite(MAX7219CS,LOW);   //CS on
  for(int i=128;i>0;i=i>>1){
    if(i&reg){
      digitalWrite(MAX7219DIN,HIGH);
    }else{
      digitalWrite(MAX7219DIN,LOW);      
    }
    digitalWrite(MAX7219CLK,HIGH);   
    digitalWrite(MAX7219CLK,LOW);   //CLK toggle    
  }
  for(int i=128;i>0;i=i>>1){
    if(i&data){
      digitalWrite(MAX7219DIN,HIGH);
    }else{
      digitalWrite(MAX7219DIN,LOW);      
    }
    digitalWrite(MAX7219CLK,HIGH);   
    digitalWrite(MAX7219CLK,LOW);   //CLK toggle    
  }
  digitalWrite(MAX7219CS,HIGH);   //CS off
}
