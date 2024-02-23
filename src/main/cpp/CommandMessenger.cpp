#pragma once
#include <string>

class CommandMessenger {
    public:  

        void setMessage(std::string message){
            myString = message;
        }

        std::string GetMessage(){
            return myString;
        }

    private:
        std::string myString;  

};
