//Vector Quaternion Matrix Extension
namespace VQME {

    export function RotateVec(lhs: Quaternion, rhs: Vec3) {

    }
    
    export function RotateQuat(lhs: Quaternion, rhs: Quaternion) {
        let nqw = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
        let nqx = lhs.w * rhs.x - lhs.x * rhs.w - lhs.y * rhs.z - lhs.z * rhs.y;
        let nqy = lhs.w * rhs.y - lhs.x * rhs.z - lhs.y * rhs.w - lhs.z * rhs.x;
        let nqz = lhs.w * rhs.z - lhs.x * rhs.y - lhs.y * rhs.x - lhs.z * rhs.w;
        return new VQME.Quaternion(nqw, nqx, nqy, nqz);
    }

    export class Vec3 {
        x: number;
        y: number;
        z: number;

        constructor(x: number, y: number, z: number) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        One = new Vec3(1, 1, 1);
        Zero = new Vec3(0, 0, 0);

        //convert to string
        ToString() {
            return "(" + this.x + ", " + this.y + ", " + this.z + ")"
        }

        //conver this to an array
        ToArray() {
            return [this.x, this.y, this.z];
        }

        //gets the magnitude/length of this vector
        Magnitude() {
            return Math.sqrt(this.SqrMagnitude());
        }

        //gets the magnitude/length of this vector squared. faster than Magnitude() ** 2
        SqrMagnitude() {
            return (this.x ** 2) + (this.y ** 2) + (this.z ** 2);
        }

        //normalise this vector (magnitude/length of 1)  
        Normalise() {
            let mag = this.Magnitude();
            this.x /= mag;
            this.y /= mag;
            this.z /= mag;
        }

        //return a normalised copy of this vector (magnitude/length of 1)
        Normalised() {
            let mag = this.Magnitude();
            return new Vec3(this.x / mag, this.y / mag, this.z / mag);
        }


        //return a copy of this vector scaled by a number
        Times(scale: number) {
            return Vec3.Multiply(this, scale);
        }

        //return a copy of this vector scaled differently on x y and z 
        VTimes(scale: Vec3) {
            return new Vec3(this.x * scale.x, this.y * scale.y, this.z * scale.z);
        }

        //scale this vector by a number
        TimesEquals(scale: number) {
            this.x *= scale;
            this.y *= scale;
            this.z *= scale;
        }

        //scale this vector differently on x y and z
        VTimesEquals(scale: Vec3) {
            this.x *= scale.x;
            this.y *= scale.y;
            this.z *= scale.z;
        }

        //add a vector to this vector
        Plus(pos: Vec3) {
            this.x += pos.x;
            this.y += pos.y;
            this.z += pos.z;
        }

        //return a copy of this vector with another vector added on
        PlusEquals(pos: Vec3) {
            return new Vec3(this.x + pos.x, this.y + pos.y, this.z + pos.z);
        }


        //add two vectors together
        static Add(lhs: Vec3, rhs: Vec3) {
            return new Vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }

        //subtract vector from a vector
        static Subtract(lhs: Vec3, rhs: Vec3) {
            return new Vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
        }

        //multiply a vector by a scalar
        static Multiply(v: Vec3, s: number) {
            return new Vec3(v.x * s, v.y * s, v.z * s);
        }

        //divide a vector by a scalar
        static Divide(v: Vec3, s: number) {
            return new Vec3(v.x / s, v.y / s, v.z / s);
        }

        //find the distance between two vectors squared (faster than Distance() ** 2)
        static SqrDistance(posA: Vec3, posB: Vec3) {
            return ((posA.x - posB.x) ** 2) + ((posA.y - posB.y) ** 2) + ((posA.z - posB.z) ** 2);
        }

        //find the distance between two vectors
        static Distance(posA: Vec3, posB: Vec3) {
            return Math.sqrt(Vec3.SqrDistance(posA, posB));
        }

        //get the dot product of two vectors
        static Dot(lhs: Vec3, rhs: Vec3) {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        //get the dot product of two vectors divided by both their magnitudes
        static DotNorm(lhs: Vec3, rhs: Vec3) {
            let doublemag = lhs.Magnitude() * rhs.Magnitude();
            return (Vec3.Dot(lhs, rhs) / doublemag);
        }

        //returns the angle between two vectors
        static Angle(lhs: Vec3, rhs: Vec3) {
            return Math.acos(Vec3.DotNorm(lhs, rhs));
        }

        //cross to be implemented   
    }

    export class Matrix {
        values: number[][];

        constructor(values: number[][]) {
            this.values = values;
        }

        static multiply(a: Matrix, b: Matrix) {
            let aNumRows = a.values.length, aNumCols = a.values[0].length,
                bNumRows = b.values.length, bNumCols = b.values[0].length
            let m: number[][] = [];  // initialize array of rows
            for (let r = 0; r < aNumRows; ++r) {
                m[r] = []; // initialize the current row
                for (let c = 0; c < bNumCols; ++c) {
                    m[r][c] = 0;             // initialize the current cell
                    for (let i = 0; i < aNumCols; ++i) {
                        m[r][c] += a.values[r][i] * b.values[i][c];
                    }
                }
            }
            return new Matrix(m);
        }

        static display(m: Matrix) {
            for (let s = 0; s < m.values.length; ++s) {
                console.log(m.values[s].join(' '));
            }
        }
    }

    export class Quaternion {
        w: number;
        x: number;
        y: number;
        z: number;

        constructor(w: number, x: number, y: number, z: number) {
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        //rotate lhs by rhs
        static Multiply(lhs: Quaternion, rhs: Quaternion) {
            let newW = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
            let newX = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
            let newY = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
            let newZ = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
            return new Quaternion(newW, newX, newY, newZ);
        }

        //create a quaternion from x y and z rotations in 3-2-1 format
        static FromEulerAngles(x: number, y: number, z: number) {
            let cz = Math.cos(z * 0.5);
            let sz = Math.sin(z * 0.5);
            let cx = Math.cos(x * 0.5);
            let sx = Math.sin(x * 0.5);
            let cy = Math.cos(y * 0.5);
            let sy = Math.sin(y * 0.5);

            let nw = cz * cx * cy + sz * sx * sy;
            let nx = sz * cx * cy - cz * sx * sy;
            let ny = cz * sx * cy + sz * cx * sy;
            let nz = cz * cx * sy - sz * sx * cy;

            return new Quaternion(nw, nx, ny, nz);
        }

        //create a quaternion from a vector3 (3-2-1)
        static FromEulerAnglesVec(vec: Vec3) {
            return Quaternion.FromEulerAngles(vec.x, vec.y, vec.z);
        }

        Magnitude() {
            return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
        }

        static Normalise(q: Quaternion) {
            let mag = q.Magnitude();
            q.w /= mag;
            q.x /= mag;
            q.y /= mag;
            q.z /= mag;
        }

        //create a vector 3 from a quaternion in 3-2-1 format
        static ToEulerAngles(q: Quaternion) {
            // roll (x-axis rotation)
            let sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            let cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            let nx = Math.atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            let sinp = Math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
            let cosp = Math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
            let ny = 2 * Math.atan2(sinp, cosp) - Math.PI / 2;

            // yaw (z-axis rotation)
            let siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            let nz = Math.atan2(siny_cosp, cosy_cosp);

            return [nx, ny, nz];
        }

        static ToArray(q: Quaternion) {
            return [q.w, q.x, q.y, q.z];
        }

        static ToRotationMatrix(quat: Quaternion) {
            let q = Quaternion.ToArray(quat);
            return new Matrix([
                [2 * (q[0] * q[0] + q[1] * q[1]) - 1,   2 * (q[1] * q[2] - q[0] * q[3]),        2 * (q[1] * q[3] + q[0] * q[2])],
                [2 * (q[1] * q[2] + q[0] * q[3]),       2 * (q[0] * q[0] + q[2] * q[2]) - 1,    2 * (q[2] * q[3] - q[0] * q[1])],
                [2 * (q[1] * q[3] - q[0] * q[2]),       2 * (q[2] * q[3] + q[0] * q[1]),        2 * (q[0] * q[0] + q[3] * q[3]) - 1],
            ]);
        }

        static Display(q: Quaternion) {
            console.log("w: " + q.w + ", x: " + q.x + ", y: " + q.y + ", z: " + q.z);
        }
    }
}
