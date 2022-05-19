int a = 0;
int s = 0;
int overflow = 0;
char userInput[50];

void setup() {
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        
        String command;
        overflow = 1;

        for (int i = 0; i < 50; i++) {
            while(Serial.available() < 1) {}
            userInput[i] = Serial.read();
            if(userInput[i] == '\n' || userInput[i] == '\0') {
                overflow = 0;
                break;
            }
        }

        if(overflow == 1) {
            return;
        }
        command = String(userInput);

        if (command.startsWith("get")) {
            command.remove(0, 4);
            if(command.startsWith("internal")) {
                command.remove(0, 9);
                Serial.println("internal working");
            } else if(command.startsWith("state")) {
                command.remove(0, 6);
                Serial.println(s);
            } else {
                Serial.println("unknown command");
            }
        } else if (command.startsWith("set")) {
            command.remove(0, 4);
            if(command.startsWith("action")) {



                command.remove(0, 7);
                a = (int)command[0] + 1 - (int)'1';

                //////////////
                int nextstate, isValid;
                if (a == 0)
                    nextstate = s - 5;
                else if (a == 1)
                    nextstate = s + 1;
                else if (a == 2)
                    nextstate = s + 5;
                else
                    nextstate = s - 1;

                if (nextstate < 0)
                    isValid = 0;
                else if (nextstate > 24)
                    isValid = 0;
                else if ((s % 5 == 4) && (a == 1))
                    isValid = 0;
                else if ((s % 5 == 0) && (a == 3))
                    isValid = 0;
                else
                    isValid = 1;

                if (isValid && s != 24) {
                    s = nextstate;
                }
                /////////////
                Serial.print("set a");
                Serial.println(a);



            } else if(command.startsWith("state")) {
                command.remove(0, 6);
                s = (int)command[0] + 1 - (int)'1';
                Serial.print("set s");
                Serial.println(s);
            } else {
                Serial.println("unknown command");
            }
        } else if (command.startsWith("g_status")) {
            Serial.println("[success] ");
        } else {
            Serial.flush();
        }
        delay(500);
    }
}