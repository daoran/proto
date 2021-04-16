/******************************************************************************
 *                                 MATHS
 ******************************************************************************/

function randf(min, max) {
  return Math.random() * (max - min) + min
}

function deg2rad(d) {
  return d * (Math.PI / 180.0);
}

function rad2deg(r) {
  return r * (180.0 / Math.PI);
}

function fltcmp(x, y) {
  if (Math.abs(x -  y) < 1e-6) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

function lerp(a, b, t) {
  return a * (1.0 - t) + b * t;
}

function sinc(x) {
  if (Math.abs(x) > 1e-6) {
    return Math.sin(x) / x;
  } else{
    let c_2 = 1.0 / 6.0;
    let c_4 = 1.0 / 120.0;
    let c_6 = 1.0 / 5040.0;
    let x_2 = x * x;
    let x_4 = x_2 * x_2;
    let x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

/******************************************************************************
 *                              LINEAR ALGEBRA
 ******************************************************************************/

class ZArray {
  constructor(m, n, data) {
    this.rows = m;
    this.cols = n;
    this.data = data;
    if (this.data === undefined) {
      this.data = [];
      for (let i = 0; i < (m * n); i++) {
        this.data.push(0.0);
      }
    }
  }

  set(i, j, val) {
    this.data[(i * this.cols) + j] = val;
  }

  val(i, j) {
    return this.data[(i * this.cols) + j];
  }

  block(rs, cs, re, ce) {
    console.assert(re > rs);
    console.assert(ce > cs);

    let sub_block = new ZArray(re - rs + 1, ce - cs + 1);
    let idx = 0;
    for (let i = rs; i <= re; i++) {
      for (let j = cs; j <= ce; j++) {
        sub_block.data[idx] = this.val(i, j);
        idx++;
      }
    }
    return sub_block;
  }

  transpose() {
    let A_t = new ZArray(this.cols, this.rows);

    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.cols; j++) {
        A_t.set(j, i, this.val(i, j));
      }
    }

    this.data = A_t.data;
  }

  add(B) {
    console.assert(this.rows == B.rows);
    console.assert(this.cols == B.cols);
    for (let idx = 0; idx < (this.rows * this.cols); idx++) {
      this.data[idx] += B.data[idx];
    }
  }

  sub(B) {
    console.assert(this.rows == B.rows);
    console.assert(this.cols == B.cols);
    for (let idx = 0; idx < (this.rows * this.cols); idx++) {
      this.data[idx] -= B.data[idx];
    }
  }

  scale(s) {
    for (let idx = 0; idx < (this.rows * this.cols); idx++) {
      this.data[idx] = s * this.data[idx];
    }
  }

  norm() {
    console.assert(this.cols == 1);
    let sum = 0.0;
    for (let i = 0; i < length; i++) {
      sum += this.data[i] * this.data[i];
    }
    return sqrt(sum);
  }

  dot(B) {
    let C = new ZArray(this.rows, B.cols);

    let A_m = this.rows;
    let A_n = this.cols;
    let B_m = B.rows;
    let B_n = B.cols;
    let C_m = A_m;
    let C_n = B_n;

    for (let i = 0; i < C_m; i++) {
      for (let j = 0; j < C_n; j++) {
        let sum = 0.0;
        for (let k = 0; k < A_n; k++) {
          sum += this.data[(i * A_n) + j] * B.data[(i * B_n) + j];
        }
        C.data[(i * C_n) + j] = sum;
      }
    }

    return C;
  }

  print(prefix) {
    let str = ""
    if (prefix !== undefined) {
      console.log("%s:\n", prefix);
      str += prefix + ":\n";
    }

    let idx = 0;
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.cols; j++) {
        str += this.data[idx].toString();
        idx++;
        if ((j + 1) < this.cols) {
          str += " ";
        }
      }
      str += "\n";
    }

    console.log(str);
  }
}

function eye(m, n) {
  console.assert(m != 0);
  console.assert(n != 0);

  let idx = 0.0;
  let A = new ZArray(m, n);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      A.data[idx] = (i == j) ? 1.0 : 0.0;
      idx++;
    }
  }
  return A;
}

function ones(m, n) {
  console.assert(m != 0);
  console.assert(n != 0);

  let idx = 0.0;
  let A = new ZArray(m, n);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      A.data[idx] = 1.0;
      idx++;
    }
  }
  return A;
}

function zeros(m, n) {
  console.assert(m != 0);
  console.assert(n != 0);

  let idx = 0.0;
  let A = new ZArray(m, n);
  for (let i = 0; i < (m * n); i++) {
    A.data[i] = 0.0;
  }
  return A;
}

/******************************************************************************
 *                              TRANSFORMS
 ******************************************************************************/

class Quaternion {
  constructor(qw, qx, qy, qz) {
    this.w = (qw !== undefined) ? qw : 1.0;
    this.x = (qx !== undefined) ? qx : 0.0;
    this.y = (qy !== undefined) ? qy : 0.0;
    this.z = (qz !== undefined) ? qz : 0.0;
  }

  data() {
    return [this.qw, this.qx, this.qy, this.qz];
  }

  to_euler() {
    let qw = this.w;
    let qx = this.x;
    let qy = this.y;
    let qz = this.z;

    let qw2 = qw * qw;
    let qx2 = qx * qx;
    let qy2 = qy * qy;
    let qz2 = qz * qz;

    let t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
    let t2 = asin(2 * (qy * qw - qx * qz));
    let t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

    return new ZArray(3, 1, [t1, t2, t3]);
  }

  to_rot() {
    let qw = this.w;
    let qx = this.x;
    let qy = this.y;
    let qz = this.z;

    let qx2 = qx * qx;
    let qy2 = qy * qy;
    let qz2 = qz * qz;
    let qw2 = qw * qw;

    /* Homogeneous form */
    C = [];
    /* -- 1st row */
    C[0] = qw2 + qx2 - qy2 - qz2;
    C[1] = 2 * (qx * qy - qw * qz);
    C[2] = 2 * (qx * qz + qw * qy);
    /* -- 2nd row */
    C[3] = 2 * (qx * qy + qw * qz);
    C[4] = qw2 - qx2 + qy2 - qz2;
    C[5] = 2 * (qy * qz - qw * qx);
    /* -- 3rd row */
    C[6] = 2 * (qx * qz - qw * qy);
    C[7] = 2 * (qy * qz + qw * qx);
    C[8] = qw2 - qx2 - qy2 + qz2;

    return new ZArray(3, 3, C);
  }

  lmul(q) {
    let pw = this.w;
    let px = this.x;
    let py = this.y;
    let pz = this.z;

    lprod = new ZArray(4, 4, [
      pw, -px, -py, -pz,
      px, pw, -pz, py,
      py, pz, pw, -px,
      pz, -py, px, pw
    ]);
    qm = new ZArray(4, 1, [q.w, q.x, q.y, q.z]);
    return lprod.dot(qm);
  }

  rmul(q) {
    let qw = q.w;
    let qx = q.x;
    let qy = q.y;
    let qz = q.z;

    rprod = new ZArray(4, 4, [
      qw, -qx, -qy, -qz,
      qx, qw, qz, -qy,
      qy, -qz, qw, qx,
      qz, qy, -qx, qw
    ]);
    pm = new ZArray(4, 1, [this.w, this.x, this.y, this.z]);
    return rprod.dot(pm);
  }

  mul(q) {
    return lmul(q);
  }
}

function euler321(euler) {
  let phi = euler[0];
  let theta = euler[1];
  let psi = euler[2];

  let C = new ZArray(3, 3);
  /* 1st row */
  C.data[0] = cos(psi) * cos(theta);
  C.data[1] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  C.data[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  /* 2nd row */
  C.data[3] = sin(psi) * cos(theta);
  C.data[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  C.data[5] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  /* 3rd row */
  C.data[6] = -sin(theta);
  C.data[7] = cos(theta) * sin(phi);
  C.data[8] = cos(theta) * cos(phi);

  return C;
}

function rot2quat(C) {
  let C00 = C.data[0]; let C01 = C.data[1]; let C02 = C.data[2];
  let C10 = C.data[3]; let C11 = C.data[4]; let C12 = C.data[5];
  let C20 = C.data[6]; let C21 = C.data[7]; let C22 = C.data[8];

  let tr = C00 + C11 + C22;
  let S = 0.0;
  let qw = 0.0;
  let qx = 0.0;
  let qy = 0.0;
  let qz = 0.0;

  if (tr > 0) {
    S = sqrt(tr+1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  return new Quaternion(qw, qx, qy, qz);
}

class TF {
  constructor(rot, trans) {
    this.C = rot;
    this.r = trans;
  }

  set_rot(C) {
    this.C = C;
  }

  set_trans(r) {
    this.r = r;
  }

  data() {
    let T = zeros(4, 4);
    let C = this.C.data;
    let r = this.r.data;
    T.data[0] = C[0]; T.data[1] = C[1]; T.data[2] = C[2]; T.data[3] = r[0];
    T.data[4] = C[3]; T.data[5] = C[4]; T.data[6] = C[5]; T.data[7] = r[1];
    T.data[8] = C[6]; T.data[9] = C[7]; T.data[10] = C[8]; T.data[11] = r[2];
    T.data[12] = 0.0; T.data[13] = 0.0; T.data[14] = 0.0; T.data[15] = 1.0;

    return T;
  }

  quat() {
    return rot2quat(this.C);
  }

  inv() {
    let T_inv = new TF();
    let C = new ZArray(3, 3, this.C.data);
    let r = new ZArray(3, 1, this.r.data);
    T_inv.C = C_inv = C.transpose();
    T_inv.r = C_inv.scale(-1).dot(r)
  }

  tf_point(p) {
    T = data();
    hp = new ZArray(4, 1, [p.data[0], p.data[1], p.data[2]]);
    return T.dot(hp);
  }

  tf_hpoint(hp) {
    T = data();
    return T.dot(hp);
  }
}

/******************************************************************************
 *                                 MAV
 ******************************************************************************/

function mav_new() {
  return {
    q_WB: new Quaternion(),
    r_WB: new ZArray(3, 1),
    v_WB: new ZArray(3, 1),
    w_WB: new ZArray(3, 1)
  };
}

/******************************************************************************
 *                              UNIT-TESTS
 ******************************************************************************/

function test_zarray_new() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);
  // console.log(A);
  console.assert(A.rows == 2);
  console.assert(A.cols == 2);
}

function test_zarray_set() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);
  A.set(0, 0, 0.0);
  A.set(1, 1, 0.0);

  console.assert(A.data[0] == 0.0);
  console.assert(A.data[1] == 2.0);
  console.assert(A.data[2] == 3.0);
  console.assert(A.data[3] == 0.0);
}

function test_zarray_val() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);
  console.assert(A.val(0, 0) == 1.0);
  console.assert(A.val(0, 1) == 2.0);
  console.assert(A.val(1, 0) == 3.0);
  console.assert(A.val(1, 1) == 4.0);
}

function test_zarray_block() {
  let A = new ZArray(3, 3, [1, 2, 3,
                            4, 5, 6,
                            7, 8, 9]);

  let block = A.block(1, 1, 2, 2);
  console.assert(block.val(0, 0) == 5.0);
  console.assert(block.val(0, 1) == 6.0);
  console.assert(block.val(1, 0) == 8.0);
  console.assert(block.val(1, 1) == 9.0);
}

function test_zarray_transpose() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);
  A.transpose();

  console.assert(A.val(0, 0) == 1.0);
  console.assert(A.val(0, 1) == 3.0);
  console.assert(A.val(1, 0) == 2.0);
  console.assert(A.val(1, 1) == 4.0);
}

function test_zarray_add() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);
  let B = new ZArray(2, 2, [1, 2, 3, 4]);
  A.add(B);

  console.assert(A.val(0, 0) == 2.0);
  console.assert(A.val(0, 1) == 4.0);
  console.assert(A.val(1, 0) == 6.0);
  console.assert(A.val(1, 1) == 8.0);
}

function test_zarray_sub() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);
  let B = new ZArray(2, 2, [1, 2, 3, 4]);
  A.sub(B);

  console.assert(A.val(0, 0) == 0.0);
  console.assert(A.val(0, 1) == 0.0);
  console.assert(A.val(1, 0) == 0.0);
  console.assert(A.val(1, 1) == 0.0);
}

function test_zarray_scale() {
  let A = new ZArray(2, 2, [1, 2, 3, 4]);

  A.scale(-1);
  console.assert(A.val(0, 0) == -1.0);
  console.assert(A.val(0, 1) == -2.0);
  console.assert(A.val(1, 0) == -3.0);
  console.assert(A.val(1, 1) == -4.0);
}

function test_zarray_dot() {
  A = new ZArray(2, 2, [1,2,3,4]);
  B = new ZArray(2, 2, [5,6,7,8]);
  // console.log(A.dot(B));
}

function test_zarray_print() {
  A = new ZArray(2, 2, [1,2,3,4]);
  // A.print();
}

test_zarray_new();
test_zarray_set();
test_zarray_val();
test_zarray_block();
test_zarray_transpose();
test_zarray_add();
test_zarray_sub();
test_zarray_scale();
test_zarray_dot();
test_zarray_print();
