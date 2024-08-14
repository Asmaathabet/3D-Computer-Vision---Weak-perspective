# Tomasi-Kanade Factorization Method

This project implements the Tomasi-Kanade factorization method for extracting affine structure and motion from 2D point correspondences. The method is commonly used in computer vision for estimating 3D structure and motion from multiple views of a scene.

## Overview

The provided code processes 2D point data to compute affine structure and motion matrices. It includes steps for centering the points, computing eigenvectors and eigenvalues, and solving for the affine structure and motion.

## Features

- **Data Preprocessing:** Normalizes the point data by centering it.
- **Eigen Decomposition:** Computes eigenvalues and eigenvectors for structure estimation.
- **Affine Structure Calculation:** Derives the affine structure matrix.
- **Affine Motion Calculation:** Computes the affine motion matrix.
- **Matrix Operations:** Includes matrix inversion, multiplication, and singular value decomposition (SVD).
- **File I/O:** Saves the resulting matrices to files for further analysis.

## Dependencies

- **OpenCV:** Required for matrix operations and file I/O. Install with:
  ```bash
  pip install opencv-python
  ```
- **C++ Compiler:** Ensure you have a C++ compiler that supports C++11 or later.

## Compilation and Execution

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/yourusername/tomasi-kanade-factorization.git
   cd tomasi-kanade-factorization
   ```

2. **Compile the Program:**
   ```bash
   g++ -o tomasi_kanade main.cpp `pkg-config --cflags --libs opencv4`
   ```

3. **Run the Program:**
   ```bash
   ./tomasi_kanade point_file
   ```
   Replace `point_file` with the path to your input file containing the 2D points.

## Code Explanation

### `getCoeffs`

```cpp
Mat getCoeffs(Point3f vec1, Point3f vec2) {
    Mat ret(1, 6, CV_32F);
    ret.at<float>(0, 0) = vec1.x * vec2.x;
    ret.at<float>(0, 1) = vec1.x * vec2.y + vec1.y * vec2.x;
    ret.at<float>(0, 2) = vec1.x * vec2.z + vec1.z * vec2.x;
    ret.at<float>(0, 3) = vec1.y * vec2.y;
    ret.at<float>(0, 4) = vec1.y * vec2.z + vec1.z * vec2.y;
    ret.at<float>(0, 5) = vec1.z * vec2.z;
    return ret;
}
```

Calculates the coefficients used to form the equations for affine structure computation from two 3D vectors.

### `saveMatDouble`

```cpp
void saveMatDouble(const char* filename, Mat matrix) {
    double* dataToSave = new double[matrix.rows * matrix.cols];
    for (int x = 0; x < matrix.cols; x++) for (int y = 0; y < matrix.rows; y++) {
        dataToSave[y * matrix.cols + x] = matrix.at<double>(y, x);
    }
    MatrixReaderWriter mrw(dataToSave, matrix.rows, matrix.cols);
    mrw.save(filename);
}
```

Saves a matrix of type `double` to a file. This is used for exporting the computed matrices.

### `main`

1. **Input File Validation:**
   Checks if the input file has at least 6 rows and columns. This ensures the data is sufficient for processing.

2. **Data Normalization:**
   Calculates the center of gravity for each set of points to normalize the data.

3. **Eigen Decomposition:**
   Computes the eigenvalues and eigenvectors of the data covariance matrix.

4. **Affine Motion Calculation:**
   Derives the affine motion matrix from the eigenvectors.

5. **Affine Structure Calculation:**
   Uses the affine motion matrix to compute the affine structure matrix.

6. **Matrix Setup and Solution:**
   Constructs and solves linear equations to obtain the final affine structure and motion matrices.

7. **SVD and Final Adjustments:**
   Applies Singular Value Decomposition (SVD) to refine the matrices. Saves the results to files.

## File Formats

- **Input File:** Must be a matrix of 2D points with at least 6 rows and columns.
- **Output Files:**
  - `AffStructure.mat`: Affine structure matrix.
  - `AffMotion.mat`: Affine motion matrix.
  - `Structure.mat`: Structure matrix.
  - `Structure.xyz`: Structure matrix in XYZ format.
  - `Motion.mat`: Motion matrix.

