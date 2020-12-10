
Eigen::Matrix<double, 6, 4> Athena::LeftFingersFK(const Eigen::Matrix<double, 1, 4> q){
    Eigen::Matrix<double, 4, 3> BFs;
    BFs << 65.90, 7.80, 4.0,
           55.92, 6.28, 4.0,
           55.92, 6.28, 4.0,
           41.04, 1.17, 4;

    for (int finger = 0; finger < 4; finger++){
        Eigen::Matrix<double, 1, 3> BF = BFs.row(finger);
    }
}
