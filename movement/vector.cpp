class vector
{
private:
    int _x;
    int _y;
    int _z;
public:
    vector(/* args */);
    float magnitude();
    float direction();
    ~vector();
};

vector::vector(/* args */)
{

}

float vector::magnitude()
{
    return sqrt(pow(_x, 2) + pow(_y, 2) + pow(_z, 2));
}

float vector::direction()
{
    return atan2(_y, _x);
}

vector::~vector()
{

}
