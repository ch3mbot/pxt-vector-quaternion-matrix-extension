//Vector Quaternion Matrix Extension
namespace VQME {
    export function RotateVector3(lhs: Quaternion, rhs: Vector3) {
        let qsame = new Quaternion(lhs.w, lhs.x, lhs.y, lhs.z);
        let qvec = new Quaternion(0, rhs.x, rhs.y, rhs.z);
        //let qinv = qsame.Conjugate();
        let qinv = new Quaternion(qsame.w, -qsame.x, -qsame.y, -qsame.z);

        let qoutq = VQME.RotateQuaternion(qsame, VQME.RotateQuaternion(qvec, qinv));
        return new Vector3(qoutq.x, qoutq.y, qoutq.z);
    }

    export function RotateQuaternion(lhs: Quaternion, rhs: Quaternion) {
        let a = lhs;
        let b = rhs;

        let nqw = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        let nqx = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        let nqy = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        let nqz = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

        return new VQME.Quaternion(nqw, nqx, nqy, nqz);
    }

    // generic add function taking vector2s or vector3s. outputs V2 or V3 accordingly. #FIXME should probably just output Vector3 every time.
    export function Add(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): Vector2 | Vector3 {
        let x = lhs.x + rhs.x;
        let y = lhs.y + rhs.y;

        let z = 0;
        let output3 = false;

        if (lhs instanceof Vector3) {
            output3 = true;
            z += lhs.z;
        }
        if (rhs instanceof Vector3) {
            output3 = true;
            z += rhs.z;
        }

        if (output3) {
            return new Vector3(x, y, z);
        }

        return new Vector2(x, y);
    }

    // add two vector3s together
    export function Add3(lhs: Vector3, rhs: Vector3): Vector3 {
        return new Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    // add two vector2s together
    export function Add2(lhs: Vector2, rhs: Vector2): Vector2 {
        return new Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
    }

    // generic subtract function taking vector2s or vector3s. outputs V2 or V3 accordingly. #FIXME should probably just output Vector3 every time.
    export function Subtract(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): Vector2 | Vector3 {
        let x = lhs.x - rhs.x;
        let y = lhs.y - rhs.y;

        let z = 0;
        let output3 = false;

        if (lhs instanceof Vector3) {
            output3 = true;
            z += lhs.z;
        }
        if (rhs instanceof Vector3) {
            output3 = true;
            z -= rhs.z;
        }

        if (output3) {
            return new Vector3(x, y, z);
        }

        return new Vector2(x, y);
    }

    // subtract vecto3r from a vecto3r
    export function Subtract3(lhs: Vector3, rhs: Vector3): Vector3 {
        return new Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    // subtract vecto2r from a vecto2r
    export function Subtract2(lhs: Vector2, rhs: Vector2): Vector2 {
        return new Vector2(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    // generic multiply function taking vector2s or vector3s. outputs V2 or V3 accordingly. #FIXME could probably just output Vector3 every time.
    export function Multiply(lhs: Vector2 | Vector3, rhs: number): Vector2 | Vector3 {
        let x = lhs.x * rhs;
        let y = lhs.y * rhs;

        let z = 0;
        let output3 = false;

        if (lhs instanceof Vector3) {
            output3 = true;
            z = lhs.z * rhs;
        }

        if (output3) {
            return new Vector3(x, y, z);
        }

        return new Vector2(x, y);
    }

    // multiply a vector3 by a scalar
    export function Multiply3(v: Vector3, s: number): Vector3 {
        return new Vector3(v.x * s, v.y * s, v.z * s);
    }

    // multiply a vector2 by a scalar
    export function Multiply2(v: Vector2, s: number): Vector2 {
        return new Vector2(v.x * s, v.y * s);
    }

    // generic divide function taking vector2s or vector3s. outputs V2 or V3 accordingly. #FIXME could probably just output Vector3 every time.
    export function Divide(lhs: Vector2 | Vector3, rhs: number): Vector2 | Vector3 {
        let x = lhs.x / rhs;
        let y = lhs.y / rhs;

        let z = 0;
        let output3 = false;

        if (lhs instanceof Vector3) {
            output3 = true;
            z = lhs.z / rhs;
        }

        if (output3) {
            return new Vector3(x, y, z);
        }

        return new Vector2(x, y);
    }

    // divide a vector3 by a scalar
    export function Divide3(v: Vector3, s: number): Vector3 {
        return new Vector3(v.x / s, v.y / s, v.z / s);
    }

    // divide a vector2 by a scalar
    export function Divide2(v: Vector2, s: number): Vector2 {
        return new Vector2(v.x / s, v.y / s);
    }

    // generic square distance function taking vector2s or vector3s.
    export function SqrDistance(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): number {
        let dx = lhs.x - rhs.x;
        let dy = lhs.y - rhs.y;
        let dz = 0;

        if (lhs instanceof Vector3)
            dz += lhs.z;
        if (rhs instanceof Vector3)
            dz -= rhs.z;

        return (dx ** 2) + (dy ** 2) + (dz ** 2);
    }

    // find the distance between two vectors squared (faster than Distance() ** 2)
    export function SqrDistance3(posA: Vector3, posB: Vector3): number {
        return ((posA.x - posB.x) ** 2) + ((posA.y - posB.y) ** 2) + ((posA.z - posB.z) ** 2);
    }

    // find the distance between two vectors squared (faster than Distance() ** 2)
    export function SqrDistance2(posA: Vector2, posB: Vector2): number {
        return ((posA.x - posB.x) ** 2) + ((posA.y - posB.y) ** 2);
    }

    // generic distance function taking vector2s or vector3s.
    export function Distance(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): number {
        return Math.sqrt(SqrDistance(lhs, rhs));
    }

    // find the distance between two vector3s
    export function Distance3(posA: Vector3, posB: Vector3): number {
        return Math.sqrt(SqrDistance3(posA, posB));
    }

    // find the distance between two vector2s
    export function Distance2(posA: Vector2, posB: Vector2): number {
        return Math.sqrt(SqrDistance2(posA, posB));
    }

    // generic dot product function taking vector2s or vector3s.
    // #FIXME should this be an overload with V2xV2 and V3xV3, or should we allow V2xV3?
    export function Dot(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): number {
        // just convert to vec3s and go from there. Dot2 is same as Dot3 with z=0.
        let v3lhs: Vector3;
        let v3rhs: Vector3;

        if (lhs instanceof Vector2)
            v3lhs = lhs.ToVector3();
        else
            v3lhs = lhs;
        if (rhs instanceof Vector2)
            v3rhs = rhs.ToVector3();
        else
            v3rhs = rhs;

        return Dot3(v3lhs, v3rhs);
    }

    // get the dot product of two vector3s 
    export function Dot3(lhs: Vector3, rhs: Vector3): number {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    // get the dot product of two vector2s 
    export function Dot2(lhs: Vector2, rhs: Vector2): number {
        return lhs.x * rhs.x + lhs.y * rhs.y;
    }

    // generic normalised dot product function taking vector2s or vector3s.
    export function DotNorm(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): number {
        let doublemag = lhs.Magnitude() * rhs.Magnitude();
        return Dot(lhs, rhs) / doublemag;
    }

    // get the dot product of two vector3s divided by both their magnitudes
    export function Dot3Norm(lhs: Vector3, rhs: Vector3): number {
        let doublemag = lhs.Magnitude() * rhs.Magnitude();
        return (Dot3(lhs, rhs) / doublemag);
    }

    // get the dot product of two vector2s divided by both their magnitudes
    export function Dot2Norm(lhs: Vector2, rhs: Vector2): number {
        let doublemag = lhs.Magnitude() * rhs.Magnitude();
        return (Dot2(lhs, rhs) / doublemag);
    }

    // get the cross product of two vector3s 
    export function Cross3(lhs: Vector3, rhs: Vector3): Vector3 {
        let nx = lhs.y * rhs.z - lhs.z * rhs.y;
        let ny = lhs.z * rhs.x - lhs.x * rhs.z;
        let nz = lhs.x * rhs.y - lhs.y * rhs.x;
        return new Vector3(nx, ny, nz);
    }

    // generic angle between function taking vector2s or vector3s. #FIXME is this rads or degrees?
    export function AngleBetween(lhs: Vector2 | Vector3, rhs: Vector2 | Vector3): number {
        return Math.acos(DotNorm(lhs, rhs));
    }

    // returns the angle between two vector3s
    export function AngleBetween3(lhs: Vector3, rhs: Vector3): number {
        return Math.acos(Dot3Norm(lhs, rhs));
    }

    // returns the angle between two vector2s
    export function AngleBetween2(lhs: Vector2, rhs: Vector2): number {
        return Math.acos(Dot2Norm(lhs, rhs));
    }

    export class Vector3 {
        x: number;
        y: number;
        z: number;

        constructor(x: number, y: number, z: number) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        static One = new Vector3(1, 1, 1);
        static Zero = new Vector3(0, 0, 0);

        // convert to string
        ToString() {
            return "(" + this.x + ", " + this.y + ", " + this.z + ")"
        }

        // conver this to an array
        ToArray() {
            return [this.x, this.y, this.z];
        }

        // distance to another vector
        DistanceTo(other: Vector3) {
            return Distance3(this, other);
        }

        // gets the magnitude/length of this vector squared. faster than Magnitude() ** 2
        SqrMagnitude() {
            return (this.x ** 2) + (this.y ** 2) + (this.z ** 2);
        }

        // gets the magnitude/length of this vector
        Magnitude() {
            return Math.sqrt(this.SqrMagnitude());
        }

        // normalise this vector (magnitude/length of 1)
        Normalise() {
            let mag = this.Magnitude();
            this.x /= mag;
            this.y /= mag;
            this.z /= mag;
        }

        // return a normalised copy of this vector (magnitude/length of 1)
        Normalised() {
            let mag = this.Magnitude();
            return new Vector3(this.x / mag, this.y / mag, this.z / mag);
        }

        // return a copy of this vector scaled by a number
        Times(scale: number) {
            return Multiply3(this, scale);
        }

        // scale this vector by a number
        TimesEquals(scale: number) {
            this.x *= scale;
            this.y *= scale;
            this.z *= scale;
        }

        // return a copy of this vector scaled differently on x y and z
        Scale(scale: Vector3) {
            return new Vector3(this.x * scale.x, this.y * scale.y, this.z * scale.z);
        }

        // scale this vector differently on x y and z
        ScaleEquals(scale: Vector3) {
            this.x *= scale.x;
            this.y *= scale.y;
            this.z *= scale.z;
        }

        // return a copy of this vector with another vector added on
        Plus(pos: Vector3) {
            return new Vector3(this.x + pos.x, this.y + pos.y, this.z + pos.z);
        }

        // add a vector to this vector
        PlusEquals(pos: Vector3) {
            this.x += pos.x;
            this.y += pos.y;
            this.z += pos.z;
        }

        // return a copy of this vector with another vector added on
        Minus(pos: Vector3) {
            return new Vector3(this.x - pos.x, this.y - pos.y, this.z - pos.z);
        }

        // add a vector to this vector
        MinusEquals(pos: Vector3) {
            this.x -= pos.x;
            this.y -= pos.y;
            this.z -= pos.z;
        }

        // dot product this vector with another
        DotWith(other: Vector3) {
            return Dot3(this, other);
        }

        // dot product another vector with this
        WithDot(other: Vector3) {
            return Dot3(other, this);
        }

        // return a 4D matrix of this vector
        FormatToV4Matrix() {
            return new Matrix([[this.x], [this.y], [this.z], [1]]);
        }
    }

    export class Vector2 {
        x: number;
        y: number;
        z: number;

        constructor(x: number, y: number) {
            this.x = x;
            this.y = y;
        }

        static One = new Vector2(1, 1);
        static Zero = new Vector2(0, 0);

        // convert to string
        ToString() {
            return "(" + this.x + ", " + this.y + ")";
        }

        // conver this to an array
        ToArray() {
            return [this.x, this.y];
        }

        // distance to another vector
        DistanceTo(other: Vector2) {
            return Distance2(this, other);
        }

        //gets the magnitude/length of this vector squared. faster than Magnitude() ** 2
        SqrMagnitude() {
            return (this.x ** 2) + (this.y ** 2);
        }

        // gets the magnitude/length of this vector
        Magnitude() {
            return Math.sqrt(this.SqrMagnitude());
        }

        // normalise this vector (magnitude/length of 1)
        Normalise() {
            let mag = this.Magnitude();
            this.x /= mag;
            this.y /= mag;
        }

        // return a normalised copy of this vector (magnitude/length of 1)
        Normalised() {
            let mag = this.Magnitude();
            return new Vector2(this.x / mag, this.y / mag);
        }


        // return a copy of this vector scaled by a number
        Times(scale: number) {
            return Multiply2(this, scale);
        }

        // scale this vector by a number
        TimesEquals(scale: number) {
            this.x *= scale;
            this.y *= scale;
            this.z *= scale;
        }

        // return a copy of this vector scaled differently on x y and z
        Scale(scale: Vector2) {
            return new Vector2(this.x * scale.x, this.y * scale.y);
        }

        // scale this vector differently on x y and z
        ScaleEquals(scale: Vector2) {
            this.x *= scale.x;
            this.y *= scale.y;
            this.z *= scale.z;
        }

        // return a copy of this vector with another vector added on
        Plus(pos: Vector2) {
            return new Vector2(this.x + pos.x, this.y + pos.y);
        }

        // add a vector to this vector
        PlusEquals(pos: Vector2) {
            this.x += pos.x;
            this.y += pos.y;
            this.z += pos.z;
        }

        // return a copy of this vector with another vector added on
        Minus(pos: Vector2) {
            return new Vector2(this.x - pos.x, this.y - pos.y);
        }

        // add a vector to this vector
        MinusEquals(pos: Vector2) {
            this.x -= pos.x;
            this.y -= pos.y;
            this.z -= pos.z;
        }
        
        ToVector3(): Vector3;
        ToVector3(param: number): Vector3;
        ToVector3(param?: number): Vector3 {
        let z = 0;
            if (param !== undefined) {
                z = param;
            } 
            return new Vector3(this.x, this.y, z);
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

        static Identity = new Quaternion(1, 0, 0, 0);

        // rotate other quaternion by this
        RotateFirst(other: Quaternion) {
            return RotateQuaternion(other, this);
        }

        // rotate this by other quaternion
        RotateSecond(other: Quaternion) {
            return RotateQuaternion(this, other);
        }

        FromEulerAngles(x: number, y: number, z: number): Quaternion;
        FromEulerAngles(angles: Vector3): Quaternion;
        FromEulerAngles(angles: [number, number, number]): Quaternion;
        FromEulerAngles(param1: number | Vector3 | [number, number, number], param2?: number, param3?: number): Quaternion {
            if (typeof param1 === 'number') {
                // parsing params
                let x = param1;
                let y = param2!;
                let z = param3!;
                
                // actually do calculation
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
            } else if (param1 instanceof Vector3) {
                return this.FromEulerAngles(param1.x, param1.y, param1.z);
            } else {
                return this.FromEulerAngles(param1[0], param1[1], param1[2]);
            }
        }
    
        Magnitude() {
            return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
        }

        // normalise this quaternion
        Normalise() {
            let mag = this.Magnitude();
            this.w /= mag;
            this.x /= mag;
            this.y /= mag;
            this.z /= mag;
        }

        // return a copy of this quaternion normalised
        Normalised() {
            let q = new Quaternion(this.w, this.x, this.y, this.z);
            let mag = q.Magnitude();
            q.w /= mag;
            q.x /= mag;
            q.y /= mag;
            q.z /= mag;
            return q;
        }

        // return a copy of this quaternion conjugated
        Conjugated() {
            return new Quaternion(this.w, -this.x, -this.y, -this.z);
        }

        // #FIXME static necessary?
        // create a vector 3 from a quaternion in 3-2-1 format
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

        // return an array of three numbers
        ToEulerAngles() {
            return Quaternion.ToEulerAngles(this);
        }

        // convert to array
        ToArray() {
            return [this.w, this.x, this.y, this.z];
        }

        // untested
        // no memory of how this works. #FIXME
        ToRotationMatrix() {
            let q = this.ToArray(); //wxyz? xyzw?
            //let q = [qte[1], qte[2], qte[3], qte[0]];
            return new Matrix([
                [2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])],
                [2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1])],
                [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1],
            ]);
        }

        // seems to work? old code. #FIXME
        ToRotationMatrix4() {
            let mat33 = this.ToRotationMatrix();
            let m3 = mat33.values;
            let mat44 = new Matrix([
                [m3[0][0], m3[0][1], m3[0][2], 0],
                [m3[1][0], m3[1][1], m3[1][2], 0],
                [m3[2][0], m3[2][1], m3[2][2], 0],
                [0, 0, 0, 1]
            ])
            return mat44;
        }

        ToString() {
            return "[" + this.w + ", " + this.x + ", " + this.y + ", " + this.z + "]";
        }
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

        ToString() {
            let outStr = "";
            for (let s = 0; s < this.values.length; ++s) {
                outStr += this.values[s].join(' ') + "\n";
            }
            return outStr;
        }
    }
}
