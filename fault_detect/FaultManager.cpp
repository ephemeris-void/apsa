enum class FaultName : uint8_t{
NONE{0U};
ENGINE_HEAT{1U};
OIL_LOW{2U};
};

enum class FaultLevel : uint8_t{

    WARNING{1U};
    CRITICAL{2U};
    EMERGENCY{3U};
};

struct FaultEntry{

       FaultName name {FaultName::NONE};
       FaultLevel level {FaultLevel::WARNING};

};

class FaultManager{

    public:
    FaultManager(FaultEntry& faults): fault{faults};

    void report(FaultName name,FaultLevel level,uint32_t current_time_stamp){

    //CHECK IF ALREADY HAVE
    for(auto &entry : Fault){
        if(entry.name==name){
            entry.count++;
            entry.time_stamp=current_time_stamp;

            if(entry.count>=CRITICAL_THREESHELD)[[unlikely]]{
                entry.level=FaultLevel::CRITICAL;
            }
        }
       return;
    }


    // IF NOT HAVE
    for(auto &entry : fault){
        if(entry.name==FaultName::NONE){
            entry.name=name;
            entry.level=level;
            entry.count{1U};
            entry.time_stamp=current_time_stamp;
        }
    }
    }
    // WORST LEVEL 
    [[nodiscard]] FaultLevel WorstLevel() const noexcept {
     
        FaultLevel worst{FaultLevel::WARNING};

          for(auto &entry : fault){

             if(entry.name==FaultName::NONE){
                continue;
             }

     if(static_cast<uint8_t>(entry.level)>static_cast<uint8_t>(worst)){
           worst = entry.level;
     }

    }
    return worst
    }

    //HAS FAULT 
    [[nodiscard]] bool has_fault(FaultName name) cosnt noexcept{
    for(auto &entry : fault){
        if(entry.name==name){
            return true;
       }
    }
    return false;
    }


    // CLEAR
    void clear(FaultName name) noexcept{

    for(auto &entry : fault){
    if(entry.name==name){
        entry=FaultName;
    }
}

}
    private:
    static constexpr CRITICAL_THREESHELD{3U};
    FaultEntry& fault{};
};