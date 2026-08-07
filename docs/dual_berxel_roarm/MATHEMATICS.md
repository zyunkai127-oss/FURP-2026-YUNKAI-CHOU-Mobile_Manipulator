# Mathematics and calibration derivation

All distances are converted to metres inside ROS unless explicitly labelled
millimetres. Quaternions use ROS `xyzw` order.

## 1. Depth statistics

For valid (non-zero) ROI depth values \(d_i\):

\[
r_{invalid}=\frac{N_{zero}}{N_{all}},\qquad
\bar d=\frac{1}{N}\sum_i d_i
\]

\[
\sigma_d=\sqrt{\frac{1}{N}\sum_i(d_i-\bar d)^2},\qquad
MAD=median(|d_i-median(d)|)
\]

The median and MAD were preferred for the marker centre because edges and
missing depth create outliers. The reported standard deviation is used later
as a fusion uncertainty estimate.

## 2. Pixel/depth back-projection

CameraInfo supplies the pinhole matrix

\[
K=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}.
\]

For marker centre \((u,v)\) and median depth \(Z\):

\[
X=\frac{(u-c_x)Z}{f_x},\qquad
Y=\frac{(v-c_y)Z}{f_y},\qquad P_c=[X,Y,Z]^T.
\]

The estimator uses the depth optical frame from CameraInfo, then applies the
saved homogeneous transform:

\[
P_b=R_{bc}P_c+t_{bc}.
\]

## 3. Checkerboard pose and reprojection error

An 8 × 6 object-point grid is defined as
\(P_{ij}=[0.025i,0.025j,0]^T\) metres. OpenCV PnP estimates \(R,t\) so that
projected points match detected sub-pixel corners. Reprojection RMSE is

\[
e_{px}=\sqrt{\frac{1}{N}\sum_i\|p_i-\pi(K(RP_i+t))\|^2}.
\]

Frames below 1 px were accepted. ArUco IDs 0–3 define board orientation so a
symmetrical checkerboard cannot silently flip by 180 degrees.

## 4. Relative camera transform

For sample \(k\), both cameras observe the same board:

\[
{}^{C_1}T_{C_2,k}={}^{C_1}T_{B,k}
({}^{C_2}T_{B,k})^{-1}.
\]

Translations are robustly averaged after outlier rejection; rotations are
averaged on the quaternion/rotation manifold. A sample is diverse only when
its board pose differs by at least 20 mm or 5°. Residuals are the translation
norm and relative rotation angle to the final transform.

## 5. Rigid correction using Kabsch/SVD

Given paired source points \(p_i\) and references \(q_i\), compute centroids
\(\bar p,\bar q\), centred matrices, and covariance

\[
H=\sum_i(p_i-\bar p)(q_i-\bar q)^T.
\]

With \(H=U\Sigma V^T\):

\[
R=V\,diag(1,1,det(VU^T))\,U^T,qquad
t=\bar q-R\bar p.
\]

The determinant constraint makes \(R\) a physical rotation rather than a
reflection. This procedure produced both `camera1_empirical_correction.yaml`
and the six-point `fused_to_robot_correction.yaml`.

Camera-1 correction (metres):

```text
R = [[ 0.99847761,  0.00555582,  0.05487792],
     [-0.00447147,  0.99979272, -0.01986240],
     [-0.05497689,  0.01958677,  0.99829550]]
t = [-0.01463351, 0.00049722, 0.00335271]
```

Fused-to-RoArm correction (translation shown in millimetres):

```text
R = [[ 0.98134014, -0.10336585, -0.16213275],
     [ 0.10078440,  0.99461662, -0.02408898],
     [ 0.16374991,  0.00729903,  0.98647488]]
t = [-10.25263208, 0.09260867, -22.17450662] mm
```

## 6. Uncertainty-weighted fusion

After correcting Camera 1, let the two depth standard deviations be
\(\sigma_1,\sigma_2\). Inverse-variance weights are

\[
w_i=\frac{1/\sigma_i^2}{1/\sigma_1^2+1/\sigma_2^2},\qquad
P_f=w_1P_1+w_2P_2.
\]

Thus the noisier camera contributes less. Fusion is emitted only for acceptable
time skew and spatial agreement. The robot-referenced output is
\(P_r=R_{rf}P_f+t_{rf}\).

## 7. Accuracy and repeatability

For predictions \(p_i\) and ground truth \(g\):

\[
e_i=p_i-g,\quad
RMSE_{3D}=\sqrt{\frac{1}{N}\sum_i\|e_i\|^2}.
\]

Repeatability instead measures deviations from the sample mean:

\[
RMSE_{repeat}=\sqrt{\frac{1}{N}\sum_i\|p_i-\bar p\|^2}.
\]

P95 is the 95th percentile of \(\|e_i\|\). A low repeatability RMSE only says
measurements cluster tightly; a constant calibration bias can still make
absolute accuracy poor. This distinction explains the sub-millimetre fused
repeatability alongside roughly 10–12 mm robot-referenced error.

## 8. Why the affine model was not selected

An affine map can artificially reduce training residuals by introducing scale
and shear. The measured design was close to planar/poorly conditioned, and the
affine result contained non-physical coupling. A rigid SE(3) transform is more
appropriate for two Cartesian coordinate frames and was therefore retained,
even though its training error was not the smallest possible number.
