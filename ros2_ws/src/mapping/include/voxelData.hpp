struct voxelData {
    unsigned char class_id;
    double confidence;
    float x,y,z;
    unsigned char r,g,b;
    voxelData(float x, float y, float z,unsigned char id, double conf, unsigned char _r, unsigned char _g, unsigned char _b) // initialization constructor
        : class_id(id), confidence(conf), r(_r), g(_g),b(_b){}
    voxelData(){}
};
