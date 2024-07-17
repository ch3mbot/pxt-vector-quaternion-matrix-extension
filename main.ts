/**
 * Vector Quaternion Matrix Extension
 * Defines Vector3, Vector2, Quaternion, Matrix, and several operations.
 * 
 * #TODO fix 'vector' and 'quaternion' capitalization in comments
 * #TDOO test efficiency of V2 vs V3 method of manipulation
 * #TODO decide if classes should have fancy comments, or if all code should use simpler comments
 * #TODO normalise quaternions before doing rotations? may be inefficient. 
 * #TODO finish VQME.rotateVector3Compound
 */
namespace MathVQ {
    /**
     * Rotate Vector3 by quaternion.
     * @param quaternion The Quaternion representing the rotation.
     * @param vector The vector to be rotated
     * @returns A new Vector3 corresponding to the rotated vector. 
     */
    export function rotateVector3(quaternion: Quaternion, vector: Vector3): Vector3 {
        let qsame = new Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        let qvec = new Quaternion(0, vector.x, vector.y, vector.z);
        //let qinv = qsame.Conjugate();
        let qinv = new Quaternion(qsame.w, -qsame.x, -qsame.y, -qsame.z);

        let qoutq = MathVQ.rotateQuaternion(qsame, MathVQ.rotateQuaternion(qvec, qinv));
        return new Vector3(qoutq.x, qoutq.y, qoutq.z);
    }

    /** Rotate a vector3 by multiple quaternions, in the order of the array. */
    //#FIXME not done yet
    export function rotateVector3Compound(vector: Vector3, ...quaternions: Quaternion[]) {
        let result = vector.clone();

        for (let i = 0; i < quaternions.length; i++) {
            let quatenrion = quaternions[i];
            let q = quatenrion.normalised();
            
        }
    }

    /**
     * Rotates a Vector2 counter clockwise.
     * @param angle The angle in radians.
     * @param vector The vector to be rotated.
     * @returns A new Vector2 corresponding to the rotated vector.
     */
    export function rotateVector2(angle: number, vector: Vector2): Vector2 {
        let cosTheta = Math.cos(angle);
        let sinTheta = Math.sin(angle);
        let x = vector.x * cosTheta - vector.y * sinTheta;
        let y = vector.x * sinTheta + vector.y * cosTheta;
        return new Vector2(x, y);
    }

    //#FIXME normalise them first?
    /**
     * Rotates a quaternion by another quaternion.
     * @param lhs Left hand side Quaternion.
     * @param rhs Right hand side Quaternion.
     * @returns lhs rotated by rhs.
     */
    export function rotateQuaternion(lhs: Quaternion, rhs: Quaternion): Quaternion {
        let a = lhs;
        let b = rhs;

        let nqw = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        let nqx = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        let nqy = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        let nqz = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

        return new Quaternion(nqw, nqx, nqy, nqz);
    }


    // #FIXME are all these overloads necessary? could just do V2 + V2 => V2, V3 + VA => V3
    export function add(lhs: Vector, rhs: Vector): Vector;                          // VA + VA => VA
    export function add(lhs: Vector2, rhs: Vector): Vector;                         // V2 + VA => VA
    export function add(lhs: Vector2, rhs: Vector2): Vector2;                       // V2 + V2 => V2
    export function add(lhs: Vector2, rhs: Vector3): Vector3;                       // V2 + V3 => V3
    export function add(lhs: Vector3, rhs: Vector): Vector3;                        // V3 + VA => V3
    export function add(lhs: Vector3, rhs: Vector2): Vector3;                       // V3 + V2 => V3
    export function add(lhs: Vector3, rhs: Vector3): Vector3;                       // V3 + V3 => V3
    /**
     * Generic add function taking Vector2s or Vector3s. 
     * Outputs Vector2 or Vector3 accordingly.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns A new vector corresponding to the two vectors added together.
     */
    export function add(lhs: Vector, rhs: Vector): Vector {
        let x = lhs.x + rhs.x;
        let y = lhs.y + rhs.y;
        let z = lhs.z + rhs.z;

        if (lhs instanceof Vector3 || rhs instanceof Vector3)
            return new Vector3(x, y, z);
        
        return new Vector2(x, y);
    }

    // #FIXME are all these overloads necessary? could just do V2 + V2 => V2, V3 + VA => V3
    /**
     * Generic subtract function taking Vector2s or Vector3s. 
     * Outputs Vector2 or Vector3 accordingly.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns A new vector corresponding to lhs minus rhs.
     */
    export function subtract(lhs: Vector, rhs: Vector): Vector;                         // VA + VA => VA
    export function subtract(lhs: Vector2, rhs: Vector): Vector;                        // V2 + VA => VA
    export function subtract(lhs: Vector2, rhs: Vector2): Vector2;                      // V2 + V2 => V2
    export function subtract(lhs: Vector2, rhs: Vector3): Vector3;                      // V2 + V3 => V3
    export function subtract(lhs: Vector3, rhs: Vector): Vector3;                       // V3 + VA => V3
    export function subtract(lhs: Vector3, rhs: Vector2): Vector3;                      // V3 + V2 => V3
    export function subtract(lhs: Vector3, rhs: Vector3): Vector3;                      // V3 + V3 => V3
    export function subtract(lhs: Vector, rhs: Vector): Vector {
        let x = lhs.x - rhs.x;
        let y = lhs.y - rhs.y;
        let z = lhs.z + rhs.z;

        if (lhs instanceof Vector3 || rhs instanceof Vector3) 
            return new Vector3(x, y, z);
        
        return new Vector2(x, y);
    }

    export function multiply(vector: Vector2, scalar: number): Vector2;
    export function multiply(vector: Vector3, scalar: number): Vector3;
    /**
     * Generic multiply function taking vector2 or vector3 and scalar.
     * Outputs Vector2 or Vector3 accordingly.
     * @param vector The vector to scale.
     * @param scalar The scalar to multiply the vector by.
     * @returns A new Vector corresponding to the vector multiplied by the scalar.
     */
    export function multiply(vector: Vector, scalar: number): Vector {
        let x = vector.x * scalar;
        let y = vector.y * scalar;

        if (vector instanceof Vector3) {
            return new Vector3(x, y, vector.z * scalar);
        }

        return new Vector2(x, y);
    }

    export function divide(vector: Vector2, scalar: number): Vector2;
    export function divide(vector: Vector3, scalar: number): Vector3;
    /**
     * Generic divide function taking vector2 or vector3 and scalar.
     * Outputs Vector2 or Vector3 accordingly.
     * @param vector The vector to scale.
     * @param scalar The scalar to divide the vector by.
     * @returns A new Vector corresponding to the vector divided by the scalar.
     */
    export function divide(vector: Vector, scalar: number): Vector {
        let x = vector.x / scalar;
        let y = vector.y / scalar;

        if (vector instanceof Vector3) {
            return new Vector3(x, y, vector.z / scalar);
        }

        return new Vector2(x, y);
    }

    /**
     * Generic square distance function taking any combination of Vector2 and Vector3. 
     * Faster than doing distance(lhs, rhs) ** 2.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns The distance between two vectors squared. 
     */
    export function sqrDistance(lhs: Vector, rhs: Vector): number {
        let dx = lhs.x - rhs.x;
        let dy = lhs.y - rhs.y;
        let dz = lhs.z - rhs.z;

        return (dx ** 2) + (dy ** 2) + (dz ** 2);
    }

    /** 
     * Generic distance function taking any combination of vector2s and vector3s. 
     * Returns the distance between two vectors. Consider if sqrDistance(lhs, rhs) is usable instead, as it is faster.
    */
    export function distance(lhs: Vector, rhs: Vector): number {
        return Math.sqrt(sqrDistance(lhs, rhs));
    }

    // #FIXME the comments for dot, dotNorm, and cross3 need improvement.
    /**
     * Generic dot product function taking any combination of vector2s and vector3s. 
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns lhs dot rhs.
     */
    export function dot(lhs: Vector, rhs: Vector): number {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    /**
     * Generic normalised dot product function taking any combination of vector2s and vector3s. 
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns lhs dot rhs normalised.
     */
    export function dotNorm(lhs: Vector, rhs: Vector): number {
        let doublemag = lhs.magnitude() * rhs.magnitude();
        return dot(lhs, rhs) / doublemag;
    }

    /**
     * Calculate the cross product of two Vector3s.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns lhs cross rhs.
     */
    export function cross3(lhs: Vector3, rhs: Vector3): Vector3 {
        let nx = lhs.y * rhs.z - lhs.z * rhs.y;
        let ny = lhs.z * rhs.x - lhs.x * rhs.z;
        let nz = lhs.x * rhs.y - lhs.y * rhs.x;
        return new Vector3(nx, ny, nz);
    }

    /**
     * Generic angle between function taking Vector2s or Vector3s.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns The angle between lhs and rhs in radians.
     */
    export function angleBetween(lhs: Vector, rhs: Vector): number {
        return Math.acos(dotNorm(lhs, rhs));
    }

}
    /** Generic vector interface so both V2 and V3 can be used interchangably. */
    export abstract class Vector {
        x: number;
        y: number;
        z: number;
        
        public abstract toString(): string;
        public abstract toArray(): number[];
        public abstract clone(): Vector;
        public abstract copy(other: Vector): void;

        /** Returns the distance to another vector. */
        public distanceTo(other: Vector): number {
            return MathVQ.distance(this, other);
        }
        
        /** Gets the magnitude/length of this vector squared. faster than Magnitude() ** 2. */
        public sqrMagnitude(): number {
            return (this.x ** 2) + (this.y ** 2) + (this.z ** 2);
        }

        /** Gets the magnitude/length of this vector. */
        public magnitude(): number {
            return Math.sqrt(this.sqrMagnitude());
        }

        /** Dot product this vector with another. */
        public dotWith(other: Vector): number {
            return MathVQ.dot(this, other);
        }

        /** Dot product another vector with this. */
        public withDot(other: Vector): number {
            return MathVQ.dot(other, this);
        }

        public abstract normalised(): Vector;
        public abstract normalise(): void;

        public abstract plus(other: Vector): Vector;
        public abstract plusEquals(other: Vector): void;
        public abstract minus(other: Vector): Vector;
        public abstract minusEquals(other: Vector): void;
        public abstract times(other: number): Vector;
        public abstract timesEquals(other: number): void;
        public abstract dividedBy(other: number): Vector;
        public abstract divideBy(other: number): void;
        public abstract scaled(other: Vector): Vector;
        public abstract scale(other: Vector): void;

    }

    /** 3 dimensional vector, with x y and z. */
    export class Vector3 extends Vector {
        public x: number;
        public y: number;
        public z: number;

        constructor(x: number, y: number, z: number) {
            super();
            this.x = x;
            this.y = y;
            this.z = z;
        }

        // #FIXME check style guide for class constants
        /** The vector (1, 1, 1). */
        static readonly One = new Vector3(1, 1, 1);
        /** The vector (0, 0, 0). */
        static readonly Zero = new Vector3(0, 0, 0);

        /** Convert to string. */
        public override toString(): string {
            return "(" + this.x + ", " + this.y + ", " + this.z + ")"
        }

        /** Convert to array of [x, y, z]. */
        public override toArray(): number[] {
            return [this.x, this.y, this.z];
        }

        /** Convert a 3 element array to a Vector3 */
        public static fromArray(values: number[]): Vector3 {
            if (values.length < 3)
                throw "Cannot create a Vector3 from an array with less than 3 elements."
            return new Vector3(values[0], values[1], values[2]);
        }

        /** Return a copy of this Vector3. */
        public override clone(): Vector3 {
            return new Vector3(this.x, this.y, this.z);
        }

        /** Copy the values of another Vector3 to this. Overrides this Vector3s values. */
        public override copy(other: Vector3): void {
            this.x = other.x;
            this.y = other.y;
            this.z = other.z;
        }

        /** Return a normalised copy of this vector (magnitude/length of 1) */
        public override normalised(): Vector3 {
            let mag = this.magnitude();
            return new Vector3(this.x / mag, this.y / mag, this.z / mag);
        }

        /** Normalise this vector. (magnitude/length of 1) */
        public override normalise(): void {
            this.copy(this.normalised());
        }

        /** Return a copy of this vector with another vector added. */
        public override plus(other: Vector): Vector3 {
            return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
        }

        /** Add another vector to this one. */
        public override plusEquals(other: Vector): void {
            this.copy(this.plus(other));
        }

        /** Return a copy of this vector with another vector subtracted. */
        public override minus(other: Vector): Vector3 {
            return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
        }

        /** Subtract another Vector from this one */
        public override minusEquals(other: Vector): void {
            this.copy(this.minus(other));
        }

        /** Return a copy of this Vector scaled by a number. */
        public override times(scale: number): Vector3 {
            return new Vector3(this.x * scale, this.y * scale, this.z * scale);
        }

        /** Scale this vector by a number. */
        public override timesEquals(scale: number): void {
            this.copy(this.times(scale));
        }

        /** Return a copy of this Vector divided by a number. */
        public override dividedBy(scale: number): Vector3 {
            return new Vector3(this.x / scale, this.y / scale, this.z / scale);
        }

        /** Dibide this vector by a number. */
        public override divideBy(scale: number): void {
            this.copy(this.dividedBy(scale));
        }

        /** Return a copy of this vector scaled differently on x y and z. */
        public override scaled(scale: Vector): Vector3 {
            let x = this.x * scale.x;
            let y = this.y * scale.y;
            let z = this.z * scale.z;
            if (scale instanceof Vector2)
                z = this.z;
            return new Vector3(x, y, z);
        }

        /** Scale this vector differently on x y and z. */
        public override scale(scale: Vector): void {
            this.copy(this.scaled(scale));
        }

        // unique to vector3? probably.

        /** Return a 4d matrix made from this vector. Format: [x, y, z, 1] */
        public formatToV4Matrix(): Matrix {
            return new Matrix([[this.x], [this.y], [this.z], [1]]);
        }

        /** Return a copy of this vector rotated by a quaternion. */
        public rotatedBy(rotation: Quaternion): Vector3 {
            return MathVQ.rotateVector3(rotation, this);
        }

        /** Rotate this vector by a quaternion. */
        public rotateBy(rotation: Quaternion): void {
            this.copy(this.rotatedBy(rotation));
        }
    }

    // #FIXME Vector3 uses this.copy(this.otherMethod()) a lot, while Vector2 doesn't. Which is better?
    export class Vector2 extends Vector {
        public x: number;
        public y: number;
        public readonly z: number = 0;

        constructor(x: number, y: number) {
            super();
            this.x = x;
            this.y = y;
        }

        // #FIXME check style guide for class constants
        static readonly One = new Vector2(1, 1);
        static readonly Zero = new Vector2(0, 0);

        /** Convert to string. */
        public override toString(): string {
            return "(" + this.x + ", " + this.y + ")";
        }

        /** Convert to array of [x, y]. */
        public override toArray(): number[] {
            return [this.x, this.y];
        }

        /** Convert a 3 element array to a Vector3 */
        public static fromArray(values: number[]): Vector2 {
            if (values.length < 2)
                throw "Cannot create a Vector2 from an array with less than 3 elements."
            return new Vector2(values[0], values[1]);
        }

        /** Return a copy of this Vector2. */
        public override clone(): Vector2 {
            return new Vector2(this.x, this.y);
        }

        /** Copy the values of another Vector2 to this. Overrides this Vector2s values. */
        public override copy(other: Vector2): void {
            this.x = other.x;
            this.y = other.y;
        }

        /** Return a normalised copy of this Vector (magnitude/length of 1) */
        public override normalised(): Vector2 {
            let mag = this.magnitude();
            return new Vector2(this.x / mag, this.y / mag);
        }

        /** Normalise this Vector. (magnitude/length of 1) */
        public override normalise(): void {
            let mag = this.magnitude();
            this.x /= mag;
            this.y /= mag;
        }

        /** Return a copy of this Vector with another Vector added. */
        public override plus(other: Vector2): Vector2
        public override plus(other: Vector3): Vector3;
        public override plus(other: Vector): Vector {
            let x = this.x - other.x;
            let y = this.y - other.y;
            let z = this.z - other.z;
            if (other instanceof Vector3)
                return new Vector3(x, y, z);
            return new Vector2(x, y);
        }

        /** Add another Vector2 from this vector. */
        public override plusEquals(pos: Vector2): void {
            this.x += pos.x;
            this.y += pos.y;
        }

        /** 
         * Return a copy of this Vector with another subtracted. 
         * Returns a Vector2 or Vector3 depending on what was subtracted. 
        */
        public override minus(other: Vector2): Vector2;
        public override minus(other: Vector3): Vector3;
        public override minus(other: Vector): Vector {
            let x = this.x - other.x;
            let y = this.y - other.y;
            let z = this.z - other.z;
            if(other instanceof Vector3)
                return new Vector3(x, y, z);
            return new Vector2(x, y);
        }

        /** Subtract another Vector2 from this vector. */
        public override minusEquals(other: Vector3): void {
            this.x -= other.x;
            this.y -= other.y;
        }

        /** Return a copy of this vector scaled by a number. */
        public override times(scale: number): Vector2 {
            return MathVQ.multiply(this, scale);
        }

        /** Scale this vector by a number. */
        public override timesEquals(scale: number): void {
            this.x *= scale;
            this.y *= scale;
        }

        /** Return a copy of this vector divided by a number. */
        public override dividedBy(scale: number): Vector2 {
            return MathVQ.divide(this, scale);
        }

        /** Divide this vector by a number. */
        public override divideBy(scale: number): void {
            this.x /= scale;
            this.y /= scale;
        }

        /** Return a copy of this vector scaled differently on x y and z. */
        public override scaled(scale: Vector): Vector2 {
            let x = this.x * scale.x;
            let y = this.y * scale.y;
            return new Vector2(x, y);
        }

        /** Scale this vector differently on x y and z. */
        public override scale(scale: Vector): void {
            this.x *= scale.x;
            this.y *= scale.y;
        }
        
        // unique to vector2.

        public toVector3(): Vector3;
        public toVector3(z: number): Vector3;
        /** Convert this Vector2 to a Vector3. */
        public toVector3(z?: number): Vector3 {
            if (z !== undefined) {
                return new Vector3(this.x, this.y, z);
            } 
            return new Vector3(this.x, this.y, 0);
        }
    }

    export class Quaternion {
        public w: number;
        public x: number;
        public y: number;
        public z: number;

        constructor(w: number, x: number, y: number, z: number) {
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        // #FIXME check style guide for class constants
        /** A Quaternion representing no rotation. */
        static readonly Identity = new Quaternion(1, 0, 0, 0);

        /** Create a 3 element array from this Quaternion in 3-2-1 format */
        public toEulerAngles(): number[] {
            let q = this;
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

        /** Create a Vector3 from this Quaternion in 3-2-1 format */
        public toEulerAnglesVector3(): Vector3 {
            let values: number[] = this.toEulerAngles();
            return Vector3.fromArray(values);
        }

        /** Convert this quaternion to an array of [w, x, y, z]. */
        public toArray(): number[] {
            return [this.w, this.x, this.y, this.z];
        }

        // untested
        // no memory of how this works. #FIXME seems to work fine? check if y inversion matters
        /** Convert this quaternion to a 3x3 rotation matrix. Not entirely tested. */
        public toRotationMatrix(): Matrix {
            let q = this.toArray(); //wxyz? xyzw?
            //let q = [qte[1], qte[2], qte[3], qte[0]];
            return new Matrix([
                [2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])],
                [2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1])],
                [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1],
            ]);
        }

        // seems to work? old code. #FIXME
        /** Convert this quaternion to a 4x4 rotation matrix. Not entirely tested. */
        public toRotationMatrix4(): Matrix {
            let mat33 = this.toRotationMatrix();
            let m3 = mat33.values;
            let mat44 = new Matrix([
                [m3[0][0], m3[0][1], m3[0][2], 0],
                [m3[1][0], m3[1][1], m3[1][2], 0],
                [m3[2][0], m3[2][1], m3[2][2], 0],
                [0, 0, 0, 1]
            ])
            return mat44;
        }

        public toString(): string {
            return "[" + this.w + ", " + this.x + ", " + this.y + ", " + this.z + "]";
        }

        public clone(): Quaternion {
            return new Quaternion(this.w, this.x, this.y, this.z);
        }

        // rotate other quaternion by this #FIXME better name?
        /** Return a new quaternion representing other rotated by this. */
        public rotateOther(other: Quaternion): Quaternion {
            return MathVQ.rotateQuaternion(other, this);
        }

        // rotate this by other quaternion #FIXME better name?
        /** Return a new quaternion representing this rotated by other. */
        public rotateThis(other: Quaternion): Quaternion {
            return MathVQ.rotateQuaternion(this, other);
        }

        public fromEulerAngles(x: number, y: number, z: number): Quaternion;
        public fromEulerAngles(angles: [number, number, number]): Quaternion;
        public fromEulerAngles(angles: Vector3): Quaternion;
        /** 
         * Construct a quaternion from rotation on each axis (3-2-1 format).
         * Takes either one number per axis, a three element array, or a Vector3.
         */
        public fromEulerAngles(param1: number | Vector3 | [number, number, number], param2?: number, param3?: number): Quaternion {
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
                return this.fromEulerAngles(param1.x, param1.y, param1.z);
            } else {
                return this.fromEulerAngles(param1[0], param1[1], param1[2]);
            }
        }
    
        // #FIXME necessary?
        /** 
         * Return the magnitude of this quaternion squared.  
         * w**2 + x**2 + y**2 + z**2. 
         */
        public sqrMagnitude(): number {
            return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
        }

        /** 
         * Return the magnitude of this quaternion. 
         * The square root of w**2 + x**2 + y**2 + z**2. 
         */
        public magnitude(): number {
            return Math.sqrt(this.w * this.w + this.x * this.x + this.y * this.y + this.z * this.z);
        }


        // #FIXME shouldnt this make sure w is positive?
        /** Normalise this quaternion. */
        public normalise(): void {
            let mag = this.magnitude();
            this.w /= mag;
            this.x /= mag;
            this.y /= mag;
            this.z /= mag;
        }

        // #FIXME add normalization to quaternion operations like rotation
        /** Return a copy of this quaternion normalised. */
        public normalised(): Quaternion {
            let q = new Quaternion(this.w, this.x, this.y, this.z);
            let mag = q.magnitude();
            q.w /= mag;
            q.x /= mag;
            q.y /= mag;
            q.z /= mag;
            return q;
        }

        // #FIXME add to a few places in code that should use this.
        // #TODO add conjugate function? useful?
        /** Return a copy of this quaternion conjugated. */
        public conjugated(): Quaternion {
            return new Quaternion(this.w, -this.x, -this.y, -this.z);
        }
    }

    export class Matrix {
        public values: number[][];

        constructor(values: number[][]) {
            this.values = values;
        }

        /** Turn this matrix into a pretty, well-formatted string */
        public toString(): string {
            let outStr = "";
            for (let s = 0; s < this.values.length; ++s) {
                outStr += this.values[s].join(' ') + "\n";
            }
            return outStr;
        }

        /** Return a copy of this matrix. */
        public clone(): Matrix {
            return new Matrix(this.values);
        }
        
        // #FIXME necessary?
        /** Override the values of this matrix with the values of another matrix. */
        public copy(other: Matrix): void {
            this.values = other.values;
        }

        /** Multiply two matrices together. */
        public static multiply(lhs: Matrix, rhs: Matrix): Matrix {
            let aNumRows = lhs.values.length, aNumCols = lhs.values[0].length,
                bNumRows = rhs.values.length, bNumCols = rhs.values[0].length
            let m: number[][] = [];  // initialize array of rows
            for (let r = 0; r < aNumRows; ++r) {
                m[r] = []; // initialize the current row
                for (let c = 0; c < bNumCols; ++c) {
                    m[r][c] = 0;             // initialize the current cell
                    for (let i = 0; i < aNumCols; ++i) {
                        m[r][c] += lhs.values[r][i] * rhs.values[i][c];
                    }
                }
            }
            return new Matrix(m);
        }

        // #FIXME necessary?
        /** Return a new matrix corresponding to this matrix multiplied by an other matrix. */
        public times(other: Matrix): Matrix {
            return Matrix.multiply(this, other);
        }

        // #FIXME necessary?
        /** Replace this matrix with the result of multiplying this matrix with an other matrix. */
        public timesEquals(other: Matrix): void {
            this.copy(this.times(other));   
        }
    }
