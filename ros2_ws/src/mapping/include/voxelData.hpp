#include <Eigen/Core>
struct voxelData {
    float x,y,z;
    unsigned char class_id;
    double confidence;
    unsigned char r,g,b;
    voxelData(float _x, float _y, float _z,unsigned char id, double conf, unsigned char _r, unsigned char _g, unsigned char _b) // initialization constructor
        : x(_x), y(_y), z(_z), class_id(id), confidence(conf), r(_r), g(_g),b(_b){}
    voxelData(){}//empty constructor

    Eigen:Vector3d toEigen() const{
        return Eigen:Vector3d(x,y,z);  
    }
};
