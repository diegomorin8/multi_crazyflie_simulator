class Matrix3x3{
  public:
    Matrix3x3();
    Matrix3x3(double* row0, double* row1, double *row2 );
    Matrix3x3(double* col0, double* col1, double *col2 );
    Matrix3x3(double x00, double x01, double x02,
              double x10, double x11, double x12,
              double x20, double x21, double x22);

  private:
    double matrix[3][3];
    double determinant;

  public:
    Matrix3x3 invert();
    double calcDeterminant();
    void setValue(int row, int col, double value);
}

Matrix3x3::Matrix3x3(){
  matrix = {{0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}};
}

Matrix3x3::Matrix3x3(double* row0, double* row1, double *row2 ){
  matrix = {{row0[0], row0[1], row0[2]},
            {row1[0], row1[1], row1[2]},
            {row2[0], row2[1], row2[2]}};
}

Matrix3x3::Matrix3x3(double* col0, double* col1, double *col2 ){
  matrix = {{col0[0], col1[0], col2[0]},
            {col0[1], col1[1], col2[1]},
            {col0[2], col1[2], col2[2]}};
}

Matrix3x3::Matrix3x3(double x00, double x01, double x02,
          double x10, double x11, double x12,
          double x20, double x21, double x22){
  matrix = {{x00, x01, x02},
            {x10, x11, x12},
            {x20, x21, x22}};
}

void Matrix3x3::setValue(int col, int row, double value){
  matrix[row][col] = value;
}
// matrix inversioon
// the result is put in Y
Matrix3x3 Matrix3x3::invert()
{
    Matrix3x3 matrix = new Matrix3x3();
    // get the determinant of a
    determinant = calcDeterminant();

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++)
            matrix.setValue(i,j,((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))/ determinant);
    }
}

double Matrix3x3::calcDeterminant(){
  double loc_determinant = 0
  for(i = 0; i < 3; i++)
        loc_determinant = loc_determinant + (matrix[0][i] * (matrix[1][(i+1)%3] * matrix[2][(i+2)%3] - matrix[1][(i+2)%3] * matrix[2][(i+1)%3]));
}
