class quaternion {
public:
  double w, i, j, k;

  quaternion(double _w=1, double _i=0, double _j=0, double _k=0) : w(_w), i(_i), j(_j), k(_k) {
  }

  void print() {
    Serial.print(w);
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(j);
    Serial.print(" ");
    Serial.print(k);
  }
  
  quaternion make_conj() {
    return quaternion(w, -i, -j, -k);
  }

  quaternion qnorm() {
    double qmag = sqrt(pow(w, 2) + pow(i, 2) + pow(j, 2) + pow(k, 2)) + 0.0;
    double w2 = w/qmag;
    double i2 = i/qmag;
    double j2 = j/qmag;
    double k2 = k/qmag;
    return quaternion(w2, i2, j2, k2);
  }

  /*
  quaternion qnorm(quaternion q) {
    Serial.print(" Unnormalised: ");
    q.print();
    double qmag = sqrt(pow(q.w, 2) + pow(q.i, 2) + pow(q.j, 2) + pow(q.k, 2)) + 0.0;
    double w2 = q.w/qmag;
    double i2 = q.i/qmag;
    double j2 = q.j/qmag;
    double k2 = q.k/qmag;
    return quaternion(w2, i2, j2, k2);
  }
  */

  quaternion qmult(quaternion q) {
    double w2 = w*q.w - i*q.i - j*q.j - k*q.k;
    double i2 = w*q.i + i*q.w + j*q.k - k*q.j;
    double j2 = w*q.j - i*q.k + j*q.w + k*q.i;
    double k2 = w*q.k + i*q.j - j*q.i + k*q.w;
    return quaternion(w2, i2, j2, k2);
  }

  quaternion qmag() {
    return qmult(make_conj());
  }

  quaternion qrotate(quaternion q) {
    return qmult(q).qnorm();
    //return qmult(q).qmult(make_conj()).qnorm();
    //return qnorm(qmult(q).qmult(make_conj()));
  }

  void conj() {
    i = -i;
    j = -j;
    k = -k;
  }
};

quaternion cos2quat(double theta, double ux, double uy, double uz) {
  double stheta = sin(theta/2);
  return quaternion(cos(theta/2), stheta*ux, stheta*uy, stheta*uz);
}

quaternion xyzrad2quat(double x, double y, double z) {
  // in radians
  double sx = sin(x/2);
  double sy = sin(y/2);
  double sz = sin(z/2);
  double cx = cos(x/2);
  double cy = cos(y/2);
  double cz = cos(z/2);

  double w = -sx*sy*sz + cx*cy*cz;
  double i = cx*sy*sz + sx*cy*cz;
  double j = -sx*cy*sz + cx*sy*cz;
  double k = sx*sy*cz + cx*cy*sz;
  return quaternion(w, i, j, k);
}

quaternion xyzdeg2quat(double x, double y, double z) {
  // in radians
  double sx = sin(PI*x/360.0);
  double sy = sin(PI*y/360.0);
  double sz = sin(PI*z/360.0);
  double cx = cos(PI*x/360.0);
  double cy = cos(PI*y/360.0);
  double cz = cos(PI*z/360.0);

  double w = -sx*sy*sz + cx*cy*cz;
  double i = cx*sy*sz + sx*cy*cz;
  double j = -sx*cy*sz + cx*sy*cz;
  double k = sx*sy*cz + cx*cy*sz;
  return quaternion(w, i, j, k);
}

quaternion zyxrad2quat(double x, double y, double z) {
  double sx = sin(x/2);
  double sy = sin(y/2);
  double sz = sin(z/2);
  double cx = cos(x/2);
  double cy = cos(y/2);
  double cz = cos(z/2);

  double w = sx*sy*sz + cx*cy*cz;
  double i = -sx*sy*cz + cx*cy*sz;
  double j = sx*cy*sz + cx*sy*cz;
  double k = -cx*sy*sz + sx*cy*cz;
  return quaternion(w, i, j, k);
}

quaternion zyxdeg2quat(double x, double y, double z) {
  double sx = sin(PI*x/360.0);
  double sy = sin(PI*y/360.0);
  double sz = sin(PI*z/360.0);
  double cx = cos(PI*x/360.0);
  double cy = cos(PI*y/360.0);
  double cz = cos(PI*z/360.0);

  double w = sx*sy*sz + cx*cy*cz;
  double i = -sx*sy*cz + cx*cy*sz;
  double j = sx*cy*sz + cx*sy*cz;
  double k = -cx*sy*sz + sx*cy*cz;
  return quaternion(w, i, j, k);
}
