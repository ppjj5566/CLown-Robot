
class leg_position{
private:
    int leg_number;
    float leg_x_position;
    float leg_y_position;
public:
    leg_position(int leg_x_position, int leg_y_position, int leg_number);
    ~leg_position();
};

leg_position::leg_position(int leg_x_position, int leg_y_position, int leg_number){
    this->leg_x_position = leg_x_position;
    this->leg_y_position = leg_y_position;
    this->leg_number = leg_number;
}

leg_position::~leg_position(){

}
