/**
* Vector Quaternion Matrix Extension
* Defines Vector3, Vector2, Quaternion, Matrix, and several operations.
* 
* #TODO fix 'vector' and 'quaternion' capitalization in comments
* #TODO decide if classes should have full comments, with @param, or if all code should use simpler comments
* #TODO normalise quaternions before doing rotations? may be inefficient. why would they ever be non-normal?
* #TODO add function to turn quaternion directly into transformation matrix, without converting to matrix in between
* #TODO examine using rotation matrices vs applying quaternions directly.
*/

namespace MathVQ {

    /** The conversion factor for radians to degrees. */
    export const Rad2Deg = 57.2957795131;
    /** The conversion factor for degrees to radians. */
    export const Deg2Rad = 0.0174532925199;

    /**
     * Rotate Vector3 by quaternion.
     * @param quaternion The Quaternion representing the rotation.
     * @param vector The vector to be rotated
     * @returns A new Vector3 corresponding to the rotated vector. 
    */
    export function rotateVector3(quaternion: Quaternion, vector: Vector3): Vector3 {
        let qsame = new Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        let qvec = new Quaternion(0, vector.x, vector.y, vector.z);
        let qinv = qsame.conjugated();

        let qoutq = MathVQ.rotateQuaternion(qsame, MathVQ.rotateQuaternion(qvec, qinv));
        return new Vector3(qoutq.x, qoutq.y, qoutq.z);
    }

    /** Rotate a vector3 by multiple quaternions, in the order of the array. */
    export function rotateVector3Compound(vector: Vector3, ...quaternions: Quaternion[]) {
        let transformationMatrix: Matrix4x4;
        transformationMatrix = Matrix4x4.Identity();
        for (let i = 0; i < quaternions.length; i++) {
            let quatenrion = quaternions[i];
            let q = quatenrion.normalised(); // #FIXME necessary? quaternions should be normalised by default.
            let m = Matrix4x4.rotationMatrix(q);
            transformationMatrix = transformationMatrix.multiply(m);
        }

        return transformationMatrix.applyToVector3(vector);
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

    /**
     * Generic square distance function taking any combination of Vector2 and Vector3. 
     * Faster than doing distance(lhs, rhs) ** 2.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns The distance between two vectors squared. 
     */
    export function sqrDistance3(lhs: Vector3, rhs: Vector3): number {
        let dx = lhs.x - rhs.x;
        let dy = lhs.y - rhs.y;
        let dz = lhs.z - rhs.z;

        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    /** 
     * Generic distance function taking any combination of vector2s and vector3s. 
     * Returns the distance between two vectors. Consider if sqrDistance(lhs, rhs) is usable instead, as it is faster.
    */
    export function distance3(lhs: Vector3, rhs: Vector3): number {
        return Math.sqrt(sqrDistance3(lhs, rhs));
    }

    // #FIXME the comments for dot, dotNorm, and cross3 need improvement.
    /**
     * Generic dot product function taking any combination of vector2s and vector3s. 
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns lhs dot rhs.
     */
    export function dot3(lhs: Vector3, rhs: Vector3): number {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }
    
    /**
     * Generic normalised dot product function taking any combination of vector2s and vector3s. 
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns lhs dot rhs normalised.
     */
    export function dot3Norm(lhs: Vector3, rhs: Vector3): number {
        let doublemag = lhs.magnitude() * rhs.magnitude();
        return dot3(lhs, rhs) / doublemag;
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
    export function angle3(lhs: Vector3, rhs: Vector3): number {
        return Math.acos(dot3Norm(lhs, rhs));
    }
}

/** 3 dimensional vector, with x y and z. */
class Vector3 {

    public x: number;
    public y: number;
    public z: number;

    constructor(x: number, y: number, z: number) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // ----- Common Vector3s -----

    /** Returns the vector (1, 1, 1). */
    public static One(): Vector3 {
        return new Vector3(1, 1, 1);
    }
    /** Returns the vector (0, 0, 0). */
    public static Zero(): Vector3 {
        return new Vector3(0, 0, 0);
    }
    /** Returns the vector (0, 0, 1). */
    public static Forward(): Vector3 {
        return new Vector3(0, 0, 1);
    }
    /** Returns the vector (0, 0, -1). */
    public static Back(): Vector3 {
        return new Vector3(0, 0, -1);
    }
    /** Returns the vector (-1, 0, 0). */
    public static Left(): Vector3 {
        return new Vector3(-1, 0, 0);
    }
    /** Returns the vector (1, 0, 0). */
    public static Right(): Vector3 {
        return new Vector3(1, 0, 0);
    }
    /** Returns the vector (0, 1, 0). */
    public static Up(): Vector3 {
        return new Vector3(0, 1, 0);
    }
    /** Returns the vector (0, -1, 0). */
    public static Down(): Vector3 {
        return new Vector3(0, -1, 0);
    }


    // ----- Vector3 Conversion Functions -----

    /** Convert to string. */
    public toString(): string {
        return "(" + this.x + ", " + this.y + ", " + this.z + ")";
    }

    /** Convert to array of [x, y, z]. */
    public toArray(): number[] {
        return [this.x, this.y, this.z];
    }

    /** Create a new Vector2 with the same x and y, dropping the z.*/
    public toVector2(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    // /** Return a 4x1 matrix corresponding to this vector. Format: [x, y, z, 1]. Transformation matrices can be multiplied with this. */
    // public to4x1Matrix(): Matrix4x {
    //     return new Matrix([[this.x], [this.y], [this.z], [1]]);
    // }

    /** Return a transformation matrix corresponding to a translation by this vector. */
    public toTranslationMatrix(): Matrix4x4 {
        return Matrix4x4.translationMatrixFromVector3(this);
    }

    /** Return a transformation matrix corresponding to scaling by this vector. */
    public toScaleMatrix(): Matrix4x4 {
        return Matrix4x4.scaleMatrixFromVec3(this);
    }

    // ----- Vector3 Creation Functions -----

    /** Convert a 3 element array to a Vector3 */
    public static fromArray(values: number[]): Vector3 {
        if (values.length < 3)
            throw "RangeError: Cannot create a Vector3 from an array with less than 3 elements.";
        return new Vector3(values[0], values[1], values[2]);
    }

    /** Return a copy of this Vector3. */
    public clone(): Vector3 {
        return new Vector3(this.x, this.y, this.z);
    }

    // #FIXME probably remove this.
    /** Copy the values of another Vector3 to this. Overrides this Vector3s values. */
    public copy(other: Vector3): void {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    // ----- Vector3 Complex Math Functions -----

    /** Returns the distance to another vector squared. Faster than distanceTo() ** 2. */
    public sqrDistanceTo(other: Vector3): number {
        let dx = other.x - this.x;
        let dy = other.y - this.y;
        let dz = other.z - this.z;

        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    /** Returns the distance to another vector. */
    public distanceTo(other: Vector3): number {
        return Math.sqrt(this.sqrDistanceTo(other));
    }

    /** Gets the magnitude/length of this vector squared. Faster than Magnitude() ** 2. */
    public sqrMagnitude(): number {
        return (this.x * this.x) + (this.y * this.y) + (this.z * this.z);
    }

    /** Gets the magnitude/length of this vector. */
    public magnitude(): number {
        return Math.sqrt(this.sqrMagnitude());
    }

    /** Dot product this vector with another. */
    public dotWith(other: Vector3): number {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    /** Dot product this vector with another normalized between -1 and 1. */
    public dotNorm(other: Vector3): number {
        return this.dotWith(other) / Math.sqrt(this.sqrMagnitude() + other.sqrMagnitude());
    }

    /** The cross product of this vector with another. */
    public cross(other: Vector3): Vector3 {
        return new Vector3(
            this.y * other.z - this.z * other.y,
            this.z * other.x - this.x * other.z,
            this.x * other.y - this.y * other.x
        );
    }

    /** Calculate the angle between this vector and another. Returns in #FIXME does acos work with rads or deg by default? */
    public angleWith(other: Vector3): number {
        return Math.acos(this.dotWith(other) / Math.sqrt(this.sqrMagnitude() * other.sqrMagnitude()))
    }


    // #FIXME rename to something like normalise and normaliseSelf for clarity?
    /** Return a copy of this vector normalised. */
    public normalised(): Vector3 {
        return this.div(this.magnitude());
    }

    // #FIXME should these comments say something like "sets length to 1"
    /** Normalise this vector. */
    public normalise(): void {
        this.divSelf(this.magnitude());
    }

    //#FIXME rename to something like scale and scaleSelf?
    /** Returns a copy of this vector scaled by another vector, differently on each axis. */
    public scaled(other: Vector3): Vector3 {
        return new Vector3(this.x * other.x, this.y * other.y, this.z * other.z);
    }

    /** Scales this vector by another vector, differently on each axis. Changes this vectors data. */
    public scale(other: Vector3): void {
        this.x *= other.x;
        this.y *= other.y;
        this.z *= other.z;
    }

    // third #FIXME of this kind. rotated & rotate or rorate & rotateSelf?
    /** Return a copy of this vector rotated by a quaternion. */
    public rotated(rotation: Quaternion): Vector3 {
        return MathVQ.rotateVector3(rotation, this);
    }

    /** Rotate this vector by a quaternion. Changes this vectors data. */
    public rotate(rotation: Quaternion): void {
        this.copy(this.rotated(rotation));
    }

    // ----- Vector3 Simple Math Functions (operations) -----

    /** Return a copy of this vector with another added. */
    public add(other: Vector3): Vector3 {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    /** Add another vector to this one. Changes this vectors data. */
    public addSelf(other: Vector3): void {
        this.x += other.x;
        this.y += other.y;
        this.z += other.z;
    }

    /** Return a copy of this vector with another subtracted. */
    public sub(other: Vector3): Vector3 {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    /** Subtract another vector from this one. Changes this vectors data. */
    public subSelf(other: Vector3): void {
        this.x -= other.x;
        this.y -= other.y;
        this.z -= other.z;
    }

    // #FIXME should this comment say 'number' instead of 'scalar'?
    /** Return a copy of this vector multiplied by a scalar. */
    public mult(scalar: number): Vector3 {
        return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    /** Multiply this vector by a scalar. Changes this vectors data. */
    public multSelf(scalar: number): void {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
    }

    /** Return a copy of this vector multiplied by a scalar. */
    public div(scalar: number): Vector3 {
        return new Vector3(this.x / scalar, this.y / scalar, this.z / scalar);
    }

    /** Divide this vector by a scalar. Changes this vectors data. */
    public divSelf(scalar: number): void {
        this.x /= scalar;
        this.y /= scalar;
        this.z /= scalar;
    }
}

class Vector2 {
    public x: number;
    public y: number;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }

    // ----- Common Vector2s -----

    /** Returns the vector (1, 1, 1). */
    public static One(): Vector2 {
        return new Vector2(1, 1);
    }
    /** Returns the vector (0, 0, 0). */
    public static Zero(): Vector2 {
        return new Vector2(0, 0);
    }
    /** Returns the vector (-1, 0, 0). */
    public static Left(): Vector2 {
        return new Vector2(-1, 0);
    }
    /** Returns the vector (1, 0, 0). */
    public static Right(): Vector2 {
        return new Vector2(1, 0);
    }
    /** Returns the vector (0, 1, 0). */
    public static Up(): Vector2 {
        return new Vector2(0, 1);
    }
    /** Returns the vector (0, -1, 0). */
    public static Down(): Vector2 {
        return new Vector2(0, -1);
    }


    // ----- Vector2 Conversion Functions -----

    /** Convert to string. */
    public toString(): string {
        return "(" + this.x + ", " + this.y + ")";
    }

    /** Convert to array of [x, y]. */
    public toArray(): number[] {
        return [this.x, this.y];
    }

    /** Create a new Vector3 with the same x and y, with a new z. Sets z to 0 if not given.*/
    public toVector3(z?: number): Vector3 {
        if (z)
            return new Vector3(this.x, this.y, z);
        return new Vector3(this.x, this.y, 0);
    }

    // /** Return a 4x1 matrix corresponding to this vector. Format: [x, y, 0, 1]. This may be removed, as it seems to have no utility.  */
    // public to4x1Matrix(): Matrix {
    //     return new Matrix([[this.x], [this.y], [0], [1]]);
    // }

    /** Return a transformation matrix corresponding to a translation by this vector. Z translation is assumed to be 0. */
    public toTranslationMatrix(): Matrix4x4 {
        return Matrix4x4.translationMatrix(this.x, this.y, 0);
    }

    /** Return a transformation matrix corresponding to scaling by this vector. Z scale is assumed to be 1. */
    public toScaleMatrix(): Matrix4x4 {
        return Matrix4x4.scaleMatrix(this.x, this.y, 1);
    }

    // ----- Vector2 Creation Functions -----

    /** Convert a 2 element array to a Vector2 */
    public static fromArray(values: number[]): Vector2 {
        if (values.length < 2)
            throw "RangeError: Cannot create a Vector2 from an array with less than 2 elements.";
        return new Vector2(values[0], values[1]);
    }

    /** Return a copy of this Vector3. */
    public clone(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    // #FIXME probably remove this.
    /** Copy the values of another Vector3 to this. Overrides this Vector3s values. */
    public copy(other: Vector2): void {
        this.x = other.x;
        this.y = other.y;
    }

    // ----- Vector2 Complex Math Functions -----

    /** Returns the distance to another vector squared. Faster than distanceTo() ** 2. */
    public sqrDistanceTo(other: Vector2): number {
        let dx = other.x - this.x;
        let dy = other.y - this.y;

        return (dx * dx) + (dy * dy);
    }

    /** Returns the distance to another vector. */
    public distanceTo(other: Vector2): number {
        return Math.sqrt(this.sqrDistanceTo(other));
    }

    /** Gets the magnitude/length of this vector squared. Faster than Magnitude() ** 2. */
    public sqrMagnitude(): number {
        return (this.x * this.x) + (this.y * this.y);
    }

    /** Gets the magnitude/length of this vector. */
    public magnitude(): number {
        return Math.sqrt(this.sqrMagnitude());
    }

    /** Dot product this vector with another. */
    public dotWith(other: Vector2): number {
        return this.x * other.x + this.y * other.y;
    }

    /** Dot product this vector with another normalized between -1 and 1. */
    public dotNorm(other: Vector2): number {
        return this.dotWith(other) / Math.sqrt(this.sqrMagnitude() + other.sqrMagnitude());
    }

    /** Calculate the angle between this vector and another. Returns in #FIXME does acos work with rads or deg by default? */
    public angleWith(other: Vector2): number {
        return Math.acos(this.dotWith(other) / Math.sqrt(this.sqrMagnitude() * other.sqrMagnitude()))
    }


    // #FIXME rename to something like normalise and normaliseSelf for clarity?
    /** Return a copy of this vector normalised. */
    public normalised(): Vector2 {
        return this.div(this.magnitude());
    }

    // #FIXME should these comments say something like "sets length to 1"
    /** Normalise this vector. */
    public normalise(): void {
        this.divSelf(this.magnitude());
    }

    //#FIXME rename to something like scale and scaleSelf?
    /** Returns a copy of this vector scaled by another vector, differently on each axis. */
    public scaled(other: Vector2): Vector2 {
        return new Vector2(this.x * other.x, this.y * other.y);
    }

    /** Scales this vector by another vector, differently on each axis. Changes this vectors data. */
    public scale(other: Vector2): void {
        this.x *= other.x;
        this.y *= other.y;
    }

    // third #FIXME of this kind. rotated & rotate or rorate & rotateSelf?
    /** Return a copy of this vector rotated by an angle in radians CCW. */
    public rotated(rads: number): Vector2 {
        let cos = Math.cos(rads);
        let sin = Math.cos(rads);
        return new Vector2(
            this.x * cos + this.y * sin,
            -this.x * sin + this.y * cos
        );
    }

    /** Rotate this vector by an angle in radians CCW. Changes this vectors data. */
    public rotate(rads: number): void {
        let cos = Math.cos(rads);
        let sin = Math.cos(rads);

        let nx = this.x * cos + this.y * sin;
        let ny = -this.x * sin + this.y * cos;

        this.x = nx;
        this.y = ny;
    }

    // ----- Vector2 Simple Math Functions (operations) -----

    /** Return a copy of this vector with another added. */
    public add(other: Vector2): Vector2 {
        return new Vector2(this.x + other.x, this.y + other.y);
    }

    /** Add another vector to this one. Changes this vectors data. */
    public addSelf(other: Vector2): void {
        this.x += other.x;
        this.y += other.y;
    }

    /** Return a copy of this vector with another subtracted. */
    public sub(other: Vector2): Vector2 {
        return new Vector2(this.x - other.x, this.y - other.y);
    }

    /** Subtract another vector from this one. Changes this vectors data. */
    public subSelf(other: Vector2): void {
        this.x -= other.x;
        this.y -= other.y;
    }

    // #FIXME should this comment say 'number' instead of 'scalar'?
    /** Return a copy of this vector multiplied by a scalar. */
    public mult(scalar: number): Vector2 {
        return new Vector2(this.x * scalar, this.y * scalar);
    }

    /** Multiply this vector by a scalar. Changes this vectors data. */
    public multSelf(scalar: number): void {
        this.x *= scalar;
        this.y *= scalar;
    }

    /** Return a copy of this vector multiplied by a scalar. */
    public div(scalar: number): Vector2 {
        return new Vector2(this.x / scalar, this.y / scalar);
    }

    /** Divide this vector by a scalar. Changes this vectors data. */
    public divSelf(scalar: number): void {
        this.x /= scalar;
        this.y /= scalar;
    }
}

class Quaternion {
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
    /** Returns a Quaternion representing no rotation. */
    public static Identity(): Quaternion {
        return new Quaternion(1, 0, 0, 0);
    }

    /** Create a 3 element array from this Quaternion in 3-2-1 format. Returns angles in radians. */
    public toEulerAnglesRad(): number[] {
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

    /** Create a 3 element array from this Quaternion in 3-2-1 format. Returns angles in degrees. */
    public toEulerAnglesDeg(): number[] {
        let radArr = this.toEulerAnglesRad();
        return [radArr[0] * MathVQ.Rad2Deg, radArr[1] * MathVQ.Rad2Deg, radArr[2] * MathVQ.Rad2Deg]
    }

    /** Create a Vector3 from this Quaternion in 3-2-1 format in degrees. */
    public toEulerAnglesVector3(): Vector3 {
        let values: number[] = this.toEulerAnglesDeg();
        return Vector3.fromArray(values);
    }

    // #FIXME why does this exist? This isn't usable with like any other functions
    /** Convert this quaternion to an array of [w, x, y, z]. Do not use with transformation matrices. */
    public toArray(): number[] {
        return [this.w, this.x, this.y, this.z];
    }

    // untested
    // no memory of how this works. #FIXME seems to work fine?
    /** 
     * Convert this quaternion to a 3x3 rotation matrix. Not entirely tested. 
     * Multiplying this matrix by a 3x1 matrix representing a Vector3 applies this rotation to it.
     */
    // public toRotationMatrix3x3(): Matrix {
    //     let q = this.toArray(); //wxyz? xyzw?
    //     //let q = [qte[1], qte[2], qte[3], qte[0]];
    //     // return new Matrix([
    //     //     [2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])],
    //     //     [2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1])],
    //     //     [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1],
    //     // ]);
    // }

    // seems to work? old code. #FIXME
    /** 
     * Convert this quaternion to a 4x4 rotation matrix. Not entirely tested. 
     * This 4x4 matrix can be multiplied by a 4x1 matrix representing a Vector3 to apply this rotation to it.
     */
    public toRotationMatrix4x4(): Matrix4x4 {
        // let q = this.toArray();
        //     return new TransformationMatrix([
        //         2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2]), 0,
        //         2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1]), 0,
        //         2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1, 0,
        //         0, 0, 0, 0
        //     ]);
        return new Matrix4x4([
            2 * (this.w * this.w + this.x * this.x) - 1, 2 * (this.x * this.y - this.w * this.z), 2 * (this.x * this.z + this.w * this.y), 0,
            2 * (this.x * this.y + this.w * this.z), 2 * (this.w * this.w + this.y * this.y) - 1, 2 * (this.y * this.z - this.w * this.x), 0,
            2 * (this.x * this.z - this.w * this.y), 2 * (this.y * this.z + this.w * this.x), 2 * (this.w * this.w + this.z * this.z) - 1, 0,
            0, 0, 0, 0
        ]);
    }

    /** Convert to string. */
    public toString(): string {
        return "[" + this.w + ", " + this.x + ", " + this.y + ", " + this.z + "]";
    }

    /** Return a new quaternion with the same data as this one. */
    public clone(): Quaternion {
        return new Quaternion(this.w, this.x, this.y, this.z);
    }

    // #FIXME necesasry? why would this be used and not just myQuaternion = other.clone();
    /** Copy the data of an other quaternion, overriding this one. */
    public copy(other: Quaternion): void {
        this.w = other.w;
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
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

    /** 
     * Construct a quaternion from a Vector3 representing rotation on each axis (3-2-1 format) in degrees.
     */
    public static fromEulerAnglesDegVec3(angles: Vector3): Quaternion {
        return this.fromEulerAnglesDeg(angles.x, angles.y, angles.z);
    }

    /** 
     * Construct a quaternion from a Vector3 representing rotation on each axis (3-2-1 format) in radians.
     */
    public static fromEulerAnglesRadVec3(angles: Vector3): Quaternion {
        return this.fromEulerAnglesRad(angles.x, angles.y, angles.z);
    }

    /** 
     * Construct a quaternion from an array representing rotation on each axis (3-2-1 format) in degrees.
     */
    public static fromEulerAnglesDegArr3(angles: [number, number, number]): Quaternion {
        return this.fromEulerAnglesDeg(angles[0], angles[1], angles[2]);
    }

    /** 
     * Construct a quaternion from an array representing rotation on each axis (3-2-1 format) in radians.
     */
    public static fromEulerAnglesRadArr3(angles: [number, number, number]): Quaternion {
        return this.fromEulerAnglesRad(angles[0], angles[1], angles[2]);
    }

    /** 
     * Construct a quaternion from an x, y, and z rotation in degrees. (3-2-1 format).
     */
    public static fromEulerAnglesDeg(x: number, y: number, z: number): Quaternion {
        return this.fromEulerAnglesRad(x * MathVQ.Rad2Deg, y * MathVQ.Rad2Deg, z * MathVQ.Rad2Deg);
    }

    /**
     * Construct a quaternion from an x, y, and z rotation in radians. (3-2-1 format).
     */
    public static fromEulerAnglesRad(x: number, y: number, z: number): Quaternion {
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

    // #FIXME should this be public? nobody should use this.
    /** 
     * Return the magnitude of this quaternion's axis. 
     * The square root of x**2 + y**2 + z**2. 
     */
    private magnitude(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    // #FIXME should functions like this return a reference to themselves, or void?
    // #FIXME when and why would this need to be normalised? this should never happen.
    /** Normalise this quaternion. */
    public normalise(): void {
        let mag = this.magnitude();
        this.x /= mag;
        this.y /= mag;
        this.z /= mag;

        // #FIXME necessary? Makes sense for w to be positive...
        if (this.w < 0) {
            this.w *= -1;
            this.x *= -1;
            this.y *= -1;
            this.z *= -1;
        }
    }

    // #FIXME add normalization to quaternion operations like rotation?
    // #FIXME this function is terrible
    /** Return a copy of this quaternion normalised. */
    public normalised(): Quaternion {
        let returnion: Quaternion = this.clone();
        returnion.normalise();
        return returnion;
    }

    // #FIXME add to a few places in code that should use this.
    // #TODO add conjugate function? useful?
    /** Return a copy of this quaternion conjugated. */
    public conjugated(): Quaternion {
        return new Quaternion(this.w, -this.x, -this.y, -this.z);
    }
}

// #TODO LU decomp, QR decomp, eigenvectors/values, Hadamard Product, pseudoinverse, rank, norm, condition number
// #FIXME should set functions return the previous value? for rows and cols

interface IMatrix {
    // getters and setters
    get(row: number, col: number): number;
    set(row: number, col: number, val: number): void;

    rows(): number;
    cols(): number;

    getRow(row: number): number[];
    getCol(col: number): number[];

    setRow(row: number, vals: number[]): void;
    setCol(col: number, vals: number[]): void;

    fill(val: number): void;

    // conversion
    clone(): IMatrix;
    equals(other: IMatrix): boolean;
    toString(): string;
    toArray(): number[];
    toArray2D(): number[][];

    // Basic math stuff. do all matrices need this?
    // add(other: IMatrix): IMatrix;
    // addSelf(other: IMatrix): void;
    // sub(other: IMatrix): IMatrix;
    // subSelf(other: IMatrix): void;
    // mult(scalar: number): IMatrix;
    // multSelf(scalar: number): void;
    // div(scalar: number): IMatrix;
    // divSelf(scalar: number): void;
    // scale(other: IMatrix): IMatrix;
    // scaleSelf(other: IMatrix): void;

    // Complex math stuff
    multiply(other: IMatrix): IMatrix;
    multiplySelf(other: IMatrix): void;

    // name transposed and transpose? #FIXME
    transpose(): IMatrix;
    transposeSelf(): void;

}

// #FIXME make abstract class? All square matrices should have a static Identity function...
interface ISquareMatrix extends IMatrix {
    determinant(): number;
    inverse(): number;
    trace(): number;
    isSymmetric(): boolean;
}

// a slow generic matrix class.
class Matrix implements IMatrix {
    private values: number[][];
    private _rows: number;
    private _cols: number;

    constructor(rows: number, cols: number, values?: number[][]) {
        this._rows = rows;
        this._cols = cols;
        this.fill(0);
        if (values) {
            this.setValues(values);
        }
    }

    // replace with matching sized values
    public setValues(values: number[][]) {
        if (values.length != this._rows) {
            throw ("Matrix Error: incorrect row size")
        }
        for (let i = 0; i < this._rows; i++) {
            if (values[i].length != this._cols) {
                throw ("Matrix Error: incorrect column size")
            }
        }
        this.values = values;
    }

    // we really providing unrestricted value access like this? #FIXME
    public getValues(): number[][] {
        return this.values;
    }

    // getters and setters
    public get(row: number, col: number): number { return this.values[row][col]; }
    public set(row: number, col: number, val: number): void { this.values[row][col] = val; }

    public rows(): number { return this._rows; }
    public cols(): number { return this._cols; }

    public getRow(row: number): number[] {
        return this.values[row];
    }
    public getCol(col: number): number[] {
        let orow = [];
        for (let i = 0; i < this._rows; i++) {
            orow.push(this.values[i][col]);
        }
        return orow;
    }

    public setRow(row: number, vals: number[]): void {
        if (vals.length != this._cols) {
            throw ("Matrix Error: badly sized row. " + vals.length + " does not match expected row size " + this._cols);
        }
        this.values[row] = vals;
    }

    public setCol(col: number, vals: number[]): void {
        if (vals.length != this._rows) {
            throw ("Matrix Error: badly sized row. " + vals.length + " does not match expected row size " + this._rows);
        }
        for (let i = 0; i < this._rows; i++) {
            this.values[i][col] = vals[col];
        }
    }

    public fill(val: number): void {
        this.values = [];
        for (let i = 0; i < this._rows; i++) {
            this.values.push([]);
            for (let j = 0; j < this._cols; j++) {
                this.values[i].push(val);
            }
        }
    }

    // conversion
    public clone(): Matrix { return new Matrix(this._rows, this._cols, this.values); }

    public equals(other: IMatrix): boolean {
        if (this._rows != other.rows() || this._cols != other.cols()) {
            return false;
        }

        for (let i = 0; i < this._rows; i++) {
            for (let j = 0; j < this._cols; j++) {
                if (this.values[i][j] != other.get(i, j)) {
                    return false;
                }
            }
        }

        return true;
    }

    // #FIXME make a nice tostring
    public toString(): string {
        throw ("unimplemented");
    }
    // #FIXME test this
    public toArray(): number[] {
        let outArr: number[] = [];
        for (let i = 0; i < this._rows; i++) {
            // for (let j = 0; j < this._cols; j++) {
            //     outArr.push(this.values[i][j]);
            // }
            outArr.concat(this.values[i])
        }
        return outArr;
    }


    // #FIXME test this
    public toArray2D(): number[][] {
        return [].concat(this.values);
    }

    // Basic math stuff
    public add(other: IMatrix): Matrix {
        this.throwErrorOnDimFail(other);
        let outMat = this.clone();
        outMat.map2((val, r, c) => val + other.get(r, c));
        return outMat;
    }
    public addSelf(other: IMatrix): void {
        this.throwErrorOnDimFail(other);
        this.map2((val, r, c) => val + other.get(r, c));
    }
    public sub(other: IMatrix): Matrix {
        this.throwErrorOnDimFail(other);
        let outMat = this.clone();
        outMat.map2((val, r, c) => val - other.get(r, c))
        return outMat;;
    }
    public subSelf(other: IMatrix): void {
        this.throwErrorOnDimFail(other);
        this.map2((val, r, c) => val - other.get(r, c));
    }
    public mult(scalar: number): Matrix {
        let outMat = this.clone();
        outMat.map2((val) => val * scalar);
        return outMat;
    }
    public multSelf(scalar: number): void {
        this.map2((val) => val * scalar);
    }
    public div(scalar: number): Matrix {
        let factor = 1 / scalar;
        return this.mult(factor);
    }
    public divSelf(scalar: number): void {
        let factor = 1 / scalar;
        this.multSelf(factor);
    }
    public scale(other: IMatrix): Matrix {
        this.throwErrorOnDimFail(other);
        let outMat = this.clone();
        outMat.map2((val, r, c) => val * other.get(r, c));
        return outMat;
    }
    public scaleSelf(other: IMatrix): void {
        this.throwErrorOnDimFail(other);
        this.map2((val, r, c) => val * other.get(r, c));
    }

    // Complex math stuff
    public multiply(other: IMatrix): Matrix {
        if (this._rows != other.cols())
            throw ("Matrix Error: matrix multiplication requires the first matrix width to match the second matrix height")

        let outMat = new Matrix(this._rows, other.cols());
        let m = this._rows;
        let n = this._cols;
        let p = other.cols();

        for (let i = 0; i < m; i++) {
            for (let j = 0; j < p; j++) {
                let sum = 0;
                for (let k = 0; k < n; k++) {
                    sum += this.values[i][k] * other.get(k, j);
                }
                outMat.values[i][j] = sum;
            }
        }

        return outMat;
    }
    public multiplySelf(other: IMatrix): void {
        this.throwErrorOnDimFail(other);
        let nmat = this.multiply(other);
        this.values = nmat.values;
        this._rows = nmat._rows;
        this._cols = nmat._cols;
    }

    // name transposed and transpose? #FIXME
    public transpose(): Matrix {
        let newVals = this.values[0].map((_, col) => this.values.map(row => row[col]));
        return new Matrix(this._cols, this._rows, newVals);
    }
    public transposeSelf(): void {
        this.values = this.values[0].map((_, col) => this.values.map(row => row[col]));
        let temp = this._rows;
        this._rows = this._cols;
        this._cols = temp;
    }

    public verifyDimensionMatch(other: IMatrix): boolean {
        return this._rows == other.rows() && this._cols == other.cols();
    }

    private throwErrorOnDimFail(other: IMatrix): void {
        if (!this.verifyDimensionMatch(other))
            throw ("Matrix Error: dimension mismatch. " + this._rows + "x" + this._cols + " matrix does not match " + other.rows() + "x" + other.cols() + " matrix.");
    }

    public map(pred: ((val: number) => number)) {
        for (let i = 0; i < this._rows; i++) {
            for (let j = 0; j < this._cols; j++) {
                this.values[i][j] = pred(this.values[i][j]);
            }
        }
    }
    public map2(pred: ((val: number, row: number, col: number) => number)) {
        for (let i = 0; i < this._rows; i++) {
            for (let j = 0; j < this._cols; j++) {
                this.values[i][j] = pred(this.values[i][j], i, j);
            }
        }
    }
}

class Matrix4x4 implements ISquareMatrix {

    private values: number[];

    constructor(values: number[]) {
        this.values = values;
    }

    public get(row: number, col: number): number { return this.values[(row << 2) + col]; }
    public set(row: number, col: number, val: number) { this.values[(row << 2) + col] - val; }

    public rows(): number { return 4; }
    public cols(): number { return 4; }

    public getRow(row: number): number[] {
        let offset = (row << 2);
        return [
            this.values[0 + offset],
            this.values[1 + offset],
            this.values[2 + offset],
            this.values[3 + offset]
        ];
    }
    public getCol(col: number): number[] {
        return [
            this.values[0 + col],
            this.values[4 + col],
            this.values[8 + col],
            this.values[12 + col],
        ];
    }

    public setRow(row: number, vals: number[]): void {
        let offset = (row << 2);
        this.values[1 + offset] = vals[0];
        this.values[2 + offset] = vals[1];
        this.values[3 + offset] = vals[2];
        this.values[4 + offset] = vals[3];
    }
    public setCol(col: number, vals: number[]): void {
        this.values[0 + col] = vals[0];
        this.values[4 + col] = vals[0];
        this.values[8 + col] = vals[0];
        this.values[12 + col] = vals[0];
    }

    // Why would you ever call this. #FIXME? make it just throw something
    public fill(val: number): void {
        this.values = [
            val, val, val, val,
            val, val, val, val,
            val, val, val, val,
            val, val, val, val
        ];
    }

    /** Returns a transformation matrix representing no change. */
    public static Identity(): Matrix4x4 {
        return new Matrix4x4(Matrix4x4.IdentityArray());
    }

    public static IdentityArray(): number[] {
        return ([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ]);
    }

    public static Blank4x4Array(): number[] {
        return ([
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0
        ]);
    }

    // conversion

    /** Return a copy of this transformation matrix */
    public clone(): Matrix4x4 {
        return new Matrix4x4(this.values);
    }


    // #FIXME test .every
    public equals(other: Matrix4x4): boolean {
        return this.values.every((val, index) => val == other.values[index]);
    }

    // #FIXME implement
    public toString(): string {
        return "unimplemented"
    }

    public toArray(): number[] {
        return [
            this.values[0], this.values[1], this.values[2], this.values[3],
            this.values[4], this.values[5], this.values[6], this.values[7],
            this.values[8], this.values[9], this.values[10], this.values[11],
            this.values[12], this.values[13], this.values[14], this.values[15]
        ]
    }

    public toArray2D(): number[][] {
        return [
            [this.values[0], this.values[1], this.values[2], this.values[3]],
            [this.values[4], this.values[5], this.values[6], this.values[7]],
            [this.values[8], this.values[9], this.values[10], this.values[11]],
            [this.values[12], this.values[13], this.values[14], this.values[15]]
        ]
    }

    //#FIXME removed all safety. no idea if this will work
    /** Convert a matrix into a transformationMatrix. Must be a 4x4 matrix. */
    public static fromMatrix(matrix: Matrix): Matrix4x4 {
        return new Matrix4x4(([] as number[])
            .concat(matrix.getValues()[0])
            .concat(matrix.getValues()[1])
            .concat(matrix.getValues()[2])
            .concat(matrix.getValues()[3])
        );
    }

    // /** Given a 3x3 matrix, adds a fourth row and col full of 0s, with a 1 in the corner. */
    // public static extend3x3Matrix(matrix: Matrix): TransformationMatrix {
    //     let m33 = matrix.values;
    //     return new TransformationMatrix([
    //         m33[0][0], m33[0][1], m33[0][2], 0,
    //         m33[1][0], m33[1][1], m33[1][2], 0,
    //         m33[2][0], m33[2][1], m33[2][2], 0,
    //                 0,         0,         0, 1
    //     ])
    // }

    /** Copy the values of an other transformation matrix. Overrides the data of this matrix. */
    public copy(other: Matrix4x4) {
        this.values = other.values;
    }

    public static translationMatrixFromVector3(data: Vector3) {
        return this.translationMatrix(data.x, data.y, data.z);
    }

    /** Create a translation matrix from x y and z. */
    public static translationMatrix(x: number, y: number, z: number): Matrix4x4 {
        return new Matrix4x4([
            1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1
        ]);
    }

    /** Create a rotation matrix from x y and z rotation. 3-2-1 rotation order. In degrees. */
    public static rotationMatrixEulerAnglesDeg(x: number, y: number, z: number): Matrix4x4 {
        return this.rotationMatrixEulerAnglesDeg(x * MathVQ.Deg2Rad, y * MathVQ.Deg2Rad, z * MathVQ.Deg2Rad);
    }

    /** Create a rotation matrix from x y and z rotation. 3-2-1 rotation order. In radians. */
    public static rotationMatrixEulerAnglesRad(x: number, y: number, z: number): Matrix4x4 {
        const sx = Math.sin(x);
        const cx = Math.cos(x);
        const sy = Math.sin(y);
        const cy = Math.cos(y);
        const sz = Math.sin(z);
        const cz = Math.cos(z);

        return new Matrix4x4([
            cy * cz, cz * sx * sy - cx * sz, sx * sz + cx * cz * sy, 0,
            cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx, 0,
            -sy, cy * sx, cx * cy, 0,
            0, 0, 0, 1
        ]);
    }

    /** Create a rotation matrix from a quaternion. */
    public static rotationMatrix(data: Quaternion): Matrix4x4 {
        return data.toRotationMatrix4x4();
    }

    public static scaleMatrixFromVec3(data: Vector3) {
        return this.scaleMatrix(data.x, data.y, data.z);
    }

    /** Generate a scale matrix with different values for each axis. */
    public static scaleMatrix(x: number, y: number, z: number): Matrix4x4 {
        return new Matrix4x4([
            x, 0, 0, 0,
            0, y, 0, 0,
            0, 0, z, 0,
            0, 0, 0, 1,
        ]);
    }

    /** 
     * Completely untested skew function. 
     * Takes the coefficients for skewing the x axis with respect to the y and z axes, then the y axis with respect to the x and z axes, then z with respect to the x and y axes. 
     * When to apply or how to use? Probably before or after scale.
     */
    public static skewMatrix(XY: number, XZ: number, YX: number, YZ: number, ZX: number, ZY: number) {
        return new Matrix4x4([
            1, XY, XZ, 0,
            YX, 1, YZ, 0,
            ZX, ZY, 1, 0,
            0, 0, 0, 1,
        ]);
    }

    /**
     * Completely untested reflection function.
     * Takes a point the plane of reflection passes through, and the normal vector of the plane. 
     * When to apply? reflecting then scaling seems to be the same as scaling then reflecting.
     * Assumes the normal and the has been normalised
     */
    public static reflectMatrix(planePoint: Vector3, planeNormal: Vector3) {
        let a = planeNormal.x;
        let b = planeNormal.y;
        let c = planeNormal.z;
        let d = -(a * planePoint.x + b * planePoint.y + c * planePoint.z);
        return new Matrix4x4([
            1 - 2 * a * a, -2 * a * b, -2 * a * c, -2 * a * d,
            -2 * a * b, 1 - 2 * b * b, -2 * b * c, -2 * b * d,
            -2 * a * c, -2 * b * c, 1 - 2 * c * c, -2 * c * d,
            0, 0, 0, 1,
        ]);
    }

    /** Create a full transformation matrix from a translation, a rotation, and a scale. */
    public static compose(_translation: Vector3, _rotation: Quaternion, _scale: Vector3): Matrix4x4 {
        let translation: Matrix4x4 = this.translationMatrix(_translation.x, _translation.y, _translation.z);
        let rotation: Matrix4x4 = this.rotationMatrix(_rotation);
        let scale: Matrix4x4 = this.scaleMatrix(_scale.x, _scale.y, _scale.z);

        // scale the object "locally", then rotate it "locally", then translate it "globally"
        let identity = Matrix4x4.Identity();
        return identity.multiply(scale).multiply(rotation).multiply(translation);
    }

    /** 
     * Apply this transformation matrix to a Vector3.
     * Faster than converting the Vector3 to a 1x4 matrix and doing matrix multiplication.
     */
    public applyToVector3(vector: Vector3): Vector3 {
        return new Vector3(
            this.values[0] * vector.x + this.values[1] * vector.y + this.values[2] * vector.z + this.values[3],
            this.values[4] * vector.x + this.values[5] * vector.y + this.values[6] * vector.z + this.values[7],
            this.values[8] * vector.x + this.values[9] * vector.y + this.values[10] * vector.z + this.values[11],
        );
    }

    /** of Matrix.multiply that specifically returns a TransformationMatrix */
    public multiply(other: Matrix4x4): Matrix4x4 {
        let outMat = new Matrix4x4(Matrix4x4.Blank4x4Array());

        for (let r = 0; r < 4; r++) {
            for (let c = 0; c < 4; c++) {
                outMat.values[r * 4 + c] =
                    this.values[(r << 2) + 0] * other.values[0 + c] +
                    this.values[(r << 2) + 1] * other.values[4 + c] +
                    this.values[(r << 2) + 2] * other.values[8 + c] +
                    this.values[(r << 2) + 3] * other.values[12 + c];
            }
        }

        return outMat;
    }

    public multiplySelf(other: Matrix4x4): void {
        let nmat = this.multiply(other);
        this.copy(nmat);
    }

    // #FIXME should we allow transposing 4x4 matrices?
    public transpose(): Matrix4x4 {
        return new Matrix4x4([
            this.values[0], this.values[4], this.values[8], this.values[12],
            this.values[1], this.values[5], this.values[9], this.values[13],
            this.values[2], this.values[6], this.values[10], this.values[14],
            this.values[3], this.values[7], this.values[11], this.values[15]
        ]);
    }

    public transposeSelf(): void {
        this.copy(this.transpose());
    }

    determinant(): number {
        throw ("Method not implemented.");
    }
    inverse(): number {
        throw ("Method not implemented.");
    }
    trace(): number {
        throw ("Method not implemented.");
    }
    isSymmetric(): boolean {
        throw ("Method not implemented.");
    }

}
