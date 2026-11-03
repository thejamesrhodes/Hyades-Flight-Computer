/* ============================================================================
 * so3_math.h  —  Small, static, allocation-free linear algebra
 *
 * Header-only (static inline) primitives shared by the estimator (iekf) and the
 * controller (control). Single precision throughout (Cortex-M4F has a single-
 * precision FPU; doubles are emulated and slow). No dynamic memory, no I/O.
 *
 * Conventions
 *   - 3-vectors: float[3].
 *   - 3x3 matrices: float[9], ROW-MAJOR  (M[r*3 + c]).
 *   - NxN matrices: float[n*n], row-major, n passed explicitly.
 *   - Quaternions: Hamilton, q = [w x y z], UNIT, maps BODY -> WORLD
 *       v_world = R(q) * v_body.
 *   - Right-multiplicative (body-frame) perturbation:  q' = q (x) exp(dtheta).
 *
 * References
 *   Sola, J. (2017) Quaternion kinematics for the error-state KF, arXiv:1711.02508.
 *   Markley & Crassidis (2014) Fundamentals of Spacecraft Attitude Det. & Ctrl.
 * ============================================================================ */
#ifndef SO3_MATH_H_
#define SO3_MATH_H_

#include <math.h>
#include <string.h>
#include <stdint.h>

/* ---------------------------------------------------------------- 3-vectors */
static inline void  v3_copy(const float a[3], float o[3]) { o[0]=a[0]; o[1]=a[1]; o[2]=a[2]; }
static inline void  v3_set (float o[3], float x, float y, float z) { o[0]=x; o[1]=y; o[2]=z; }
static inline void  v3_add (const float a[3], const float b[3], float o[3]) { o[0]=a[0]+b[0]; o[1]=a[1]+b[1]; o[2]=a[2]+b[2]; }
static inline void  v3_sub (const float a[3], const float b[3], float o[3]) { o[0]=a[0]-b[0]; o[1]=a[1]-b[1]; o[2]=a[2]-b[2]; }
static inline void  v3_scale(const float a[3], float s, float o[3]) { o[0]=a[0]*s; o[1]=a[1]*s; o[2]=a[2]*s; }
static inline float v3_dot (const float a[3], const float b[3]) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
static inline float v3_norm(const float a[3]) { return sqrtf(v3_dot(a,a)); }
static inline void  v3_cross(const float a[3], const float b[3], float o[3]) {
    o[0]=a[1]*b[2]-a[2]*b[1];
    o[1]=a[2]*b[0]-a[0]*b[2];
    o[2]=a[0]*b[1]-a[1]*b[0];
}
/* Normalise in place; returns the original norm (0 if degenerate, left unchanged). */
static inline float v3_normalize(float a[3]) {
    float n = v3_norm(a);
    if (n > 1e-9f) { float inv = 1.0f/n; a[0]*=inv; a[1]*=inv; a[2]*=inv; }
    return n;
}

/* ---------------------------------------------------------------- 3x3 (row-major) */
static inline void m3_identity(float M[9]) {
    M[0]=1; M[1]=0; M[2]=0; M[3]=0; M[4]=1; M[5]=0; M[6]=0; M[7]=0; M[8]=1;
}
/* Skew-symmetric (cross-product) matrix [v]x. */
static inline void m3_skew(const float v[3], float S[9]) {
    S[0]= 0.0f; S[1]=-v[2]; S[2]= v[1];
    S[3]= v[2]; S[4]= 0.0f; S[5]=-v[0];
    S[6]=-v[1]; S[7]= v[0]; S[8]= 0.0f;
}
/* vee of the antisymmetric part: 0.5*(M - M^T) -> vector. */
static inline void m3_vee_antisym(const float M[9], float v[3]) {
    v[0] = 0.5f*(M[7]-M[5]);
    v[1] = 0.5f*(M[2]-M[6]);
    v[2] = 0.5f*(M[3]-M[1]);
}
static inline void m3_transpose(const float M[9], float T[9]) {
    T[0]=M[0]; T[1]=M[3]; T[2]=M[6];
    T[3]=M[1]; T[4]=M[4]; T[5]=M[7];
    T[6]=M[2]; T[7]=M[5]; T[8]=M[8];
}
static inline void m3_mul(const float A[9], const float B[9], float C[9]) {
    C[0]=A[0]*B[0]+A[1]*B[3]+A[2]*B[6]; C[1]=A[0]*B[1]+A[1]*B[4]+A[2]*B[7]; C[2]=A[0]*B[2]+A[1]*B[5]+A[2]*B[8];
    C[3]=A[3]*B[0]+A[4]*B[3]+A[5]*B[6]; C[4]=A[3]*B[1]+A[4]*B[4]+A[5]*B[7]; C[5]=A[3]*B[2]+A[4]*B[5]+A[5]*B[8];
    C[6]=A[6]*B[0]+A[7]*B[3]+A[8]*B[6]; C[7]=A[6]*B[1]+A[7]*B[4]+A[8]*B[7]; C[8]=A[6]*B[2]+A[7]*B[5]+A[8]*B[8];
}
static inline void m3_mulvec(const float M[9], const float v[3], float o[3]) {
    o[0]=M[0]*v[0]+M[1]*v[1]+M[2]*v[2];
    o[1]=M[3]*v[0]+M[4]*v[1]+M[5]*v[2];
    o[2]=M[6]*v[0]+M[7]*v[1]+M[8]*v[2];
}
/* o = M^T * v  (i.e. world->body when M is body->world). */
static inline void m3_T_mulvec(const float M[9], const float v[3], float o[3]) {
    o[0]=M[0]*v[0]+M[3]*v[1]+M[6]*v[2];
    o[1]=M[1]*v[0]+M[4]*v[1]+M[7]*v[2];
    o[2]=M[2]*v[0]+M[5]*v[1]+M[8]*v[2];
}
/* Analytic 3x3 inverse; returns 0 if singular (Ainv untouched). */
static inline int m3_inverse(const float A[9], float Ainv[9]) {
    float det = A[0]*(A[4]*A[8]-A[5]*A[7])
              - A[1]*(A[3]*A[8]-A[5]*A[6])
              + A[2]*(A[3]*A[7]-A[4]*A[6]);
    if (fabsf(det) < 1e-20f) { return 0; }
    float id = 1.0f/det;
    Ainv[0]= (A[4]*A[8]-A[5]*A[7])*id; Ainv[1]=-(A[1]*A[8]-A[2]*A[7])*id; Ainv[2]= (A[1]*A[5]-A[2]*A[4])*id;
    Ainv[3]=-(A[3]*A[8]-A[5]*A[6])*id; Ainv[4]= (A[0]*A[8]-A[2]*A[6])*id; Ainv[5]=-(A[0]*A[5]-A[2]*A[3])*id;
    Ainv[6]= (A[3]*A[7]-A[4]*A[6])*id; Ainv[7]=-(A[0]*A[7]-A[1]*A[6])*id; Ainv[8]= (A[0]*A[4]-A[1]*A[3])*id;
    return 1;
}

/* ---------------------------------------------------------------- quaternions */
static inline void q_normalize(float q[4]) {
    float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n > 1e-12f) { float inv=1.0f/n; q[0]*=inv; q[1]*=inv; q[2]*=inv; q[3]*=inv; }
}
/* Hamilton product r = p (x) q. */
static inline void q_mul(const float p[4], const float q[4], float r[4]) {
    r[0]=p[0]*q[0]-p[1]*q[1]-p[2]*q[2]-p[3]*q[3];
    r[1]=p[0]*q[1]+p[1]*q[0]+p[2]*q[3]-p[3]*q[2];
    r[2]=p[0]*q[2]-p[1]*q[3]+p[2]*q[0]+p[3]*q[1];
    r[3]=p[0]*q[3]+p[1]*q[2]-p[2]*q[1]+p[3]*q[0];
}
/* Exponential map so(3) -> S^3: unit quaternion from a rotation vector (rad).
 * Exact; small-angle branch avoids 0/0. This IS the Rodrigues exp used for both
 * propagation (rv = omega*dt) and the EKF reset (rv = dtheta). */
static inline void q_from_rotvec(const float rv[3], float q[4]) {
    float th = sqrtf(rv[0]*rv[0]+rv[1]*rv[1]+rv[2]*rv[2]);
    if (th < 1e-7f) {
        q[0]=1.0f; q[1]=0.5f*rv[0]; q[2]=0.5f*rv[1]; q[3]=0.5f*rv[2];  /* 1st order */
    } else {
        float s = sinf(0.5f*th)/th;
        q[0]=cosf(0.5f*th); q[1]=rv[0]*s; q[2]=rv[1]*s; q[3]=rv[2]*s;
    }
    q_normalize(q);
}
/* Rotation matrix (body->world) from a unit quaternion. */
static inline void q_to_R(const float q[4], float R[9]) {
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0]=1.0f-2.0f*(y*y+z*z); R[1]=2.0f*(x*y-w*z);      R[2]=2.0f*(x*z+w*y);
    R[3]=2.0f*(x*y+w*z);      R[4]=1.0f-2.0f*(x*x+z*z); R[5]=2.0f*(y*z-w*x);
    R[6]=2.0f*(x*z-w*y);      R[7]=2.0f*(y*z+w*x);      R[8]=1.0f-2.0f*(x*x+y*y);
}
/* Euler angles (degrees), aerospace ZYX (yaw-pitch-roll) from quaternion.
 * NOTE: roll/pitch/yaw here are the standard 3-2-1 angles; with body Z as the
 * longitudinal axis they are for human/navball readout, not control (control is
 * fully quaternion/SO(3)). */
static inline void q_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw) {
    float w=q[0], x=q[1], y=q[2], z=q[3];
    const float r2d = 57.29577951308232f;
    float sinp = 2.0f*(w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * r2d;
    *pitch = asinf(sinp) * r2d;
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)) * r2d;
}

/* ---------------------------------------------------------------- generic NxN (row-major) */
static inline void matN_zero(float *A, int n)            { memset(A, 0, (size_t)(n*n)*sizeof(float)); }
static inline void matN_identity(float *A, int n) {
    matN_zero(A, n);
    for (int i=0;i<n;i++) { A[i*n+i]=1.0f; }
}
static inline void matN_add(const float *A, const float *B, float *C, int n) {
    for (int i=0;i<n*n;i++) { C[i]=A[i]+B[i]; }
}
static inline void matN_transpose(const float *A, float *T, int n) {
    for (int i=0;i<n;i++) for (int j=0;j<n;j++) { T[j*n+i]=A[i*n+j]; }
}
static inline void matN_mul(const float *A, const float *B, float *C, int n) {
    for (int i=0;i<n;i++) for (int j=0;j<n;j++) {
        float s=0.0f;
        for (int k=0;k<n;k++) { s += A[i*n+k]*B[k*n+j]; }
        C[i*n+j]=s;
    }
}
/* Symmetrise in place: A = 0.5*(A + A^T). Keeps covariance numerically symmetric. */
static inline void matN_symmetrize(float *A, int n) {
    for (int i=0;i<n;i++) for (int j=i+1;j<n;j++) {
        float m = 0.5f*(A[i*n+j]+A[j*n+i]);
        A[i*n+j]=m; A[j*n+i]=m;
    }
}

#endif /* SO3_MATH_H_ */
