//Pin assignmentss
#define ClockwiseWinch 2
#define AntiClockwiseWinch 3

void setup() {
  // put your setup code here, to run once:
  pinMode(ClockwiseWinch, OUTPUT);
  pinMode(AntiClockwiseWinch, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(ClockwiseWinch, HIGH);
digitalWrite(AntiClockwiseWinch, LOW);

}
