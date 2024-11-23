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
        let transformationMatrix: TransformationMatrix;
        transformationMatrix = TransformationMatrix.Identity();
        for (let i = 0; i < quaternions.length; i++) {
            let quatenrion = quaternions[i];
            let q = quatenrion.normalised(); // #FIXME necessary? quaternions should be normalised by default.
            let m = TransformationMatrix.rotationMatrix(q);
            transformationMatrix = transformationMatrix.thenApply(m);
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

    export function subtract(lhs: Vector, rhs: Vector): Vector;                         // VA + VA => VA
    export function subtract(lhs: Vector2, rhs: Vector): Vector;                        // V2 + VA => VA
    export function subtract(lhs: Vector2, rhs: Vector2): Vector2;                      // V2 + V2 => V2
    export function subtract(lhs: Vector2, rhs: Vector3): Vector3;                      // V2 + V3 => V3
    export function subtract(lhs: Vector3, rhs: Vector): Vector3;                       // V3 + VA => V3
    export function subtract(lhs: Vector3, rhs: Vector2): Vector3;                      // V3 + V2 => V3
    export function subtract(lhs: Vector3, rhs: Vector3): Vector3;                      // V3 + V3 => V3

    // #FIXME are all these overloads necessary? could just do V2 + V2 => V2, V3 + VA => V3
    /**
     * Generic subtract function taking Vector2s or Vector3s. 
     * Outputs Vector2 or Vector3 accordingly.
     * @param lhs Left hand side vector.
     * @param rhs Right hand side vector.
     * @returns A new vector corresponding to lhs minus rhs.
     */
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

    export function createVector3(
        x: number,
        y: number,
        z: number,
    ): Vector3 {
        return new Vector3(x, y, z);
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

    /** Return a 4x1 matrix corresponding to this vector. Format: [x, y, z, 1]. Transformation matrices can be multiplied with this. */
    public to4x1Matrix(): Matrix {
        return new Matrix([[this.x], [this.y], [this.z], [1]]);
    }

    /** Return a transformation matrix corresponding to a translation by this vector. */
    public toTranslationMatrix(): TransformationMatrix {
        return TransformationMatrix.translationMatrix(this);
    }

    /** Return a transformation matrix corresponding to scaling by this vector. */
    public toScaleMatrix(): TransformationMatrix {
        return TransformationMatrix.scaleMatrix(this);
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

    /** Return a 4x1 matrix corresponding to this vector. Format: [x, y, 0, 1]. This may be removed, as it seems to have no utility.  */
    public to4x1Matrix(): Matrix {
        return new Matrix([[this.x], [this.y], [0], [1]]);
    }

    /** Return a transformation matrix corresponding to a translation by this vector. Z translation is assumed to be 0. */
    public toTranslationMatrix(): TransformationMatrix {
        return TransformationMatrix.translationMatrix(this);
    }

    /** Return a transformation matrix corresponding to scaling by this vector. Z scale is assumed to be 1. */
    public toScaleMatrix(): TransformationMatrix {
        return TransformationMatrix.scaleMatrix(this);
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
        return [radArr[0] * MathVQ.Rad2Deg, radArr[1] * MathVQ.Rad2Deg,radArr[2] * MathVQ.Rad2Deg]
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
    public toRotationMatrix3x3(): Matrix {
        let q = this.toArray(); //wxyz? xyzw?
        //let q = [qte[1], qte[2], qte[3], qte[0]];
        return new Matrix([
            [2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])],
            [2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1])],
            [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1],
        ]);
    }

    // seems to work? old code. #FIXME
    /** 
     * Convert this quaternion to a 4x4 rotation matrix. Not entirely tested. 
     * This 4x4 matrix can be multiplied by a 4x1 matrix representing a Vector3 to apply this rotation to it.
     */
    public toRotationMatrix4x4(): TransformationMatrix {
        return TransformationMatrix.extend3x3Matrix(this.toRotationMatrix3x3());
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


/** A number[][] type for a transformation matrix. */
type internalData4x4 = [
    [number, number, number, number],
    [number, number, number, number],
    [number, number, number, number],
    [number, number, number, number]
];

/** A number array for a transformation matrix. */
//#TODO refactor all matrix code to work with a 1d array...
type internalData4x4Lin = [
    number, number, number, number,
    number, number, number, number,
    number, number, number, number,
    number, number, number, number,
]

// const defaultInternalData4x4: internalData4x4 = [
//     [0, 0, 0, 0],
//     [0, 0, 0, 0],
//     [0, 0, 0, 0],
//     [0, 0, 0, 0],
// ]


class TransformationMatrix extends Matrix<internalData4x4Lin> {
    /** Returns a transformation matrix representing no change. */
    public static Identity(): TransformationMatrix {
        return new TransformationMatrix([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ]);
    }

    constructor(values: internalData4x4Lin) {
        super(values, 4, 4);
        this.values = values;
    }

    //#FIXME removed all safety
    /** Convert a matrix into a transformationMatrix. Must be a 4x4 matrix. */
    public static fromMatrix(matrix: Matrix): TransformationMatrix {
        // let values = matrix.values;
        // let internalData: internalData4x4;
        // if (values.length != 4)
        //     throw "RangeError: Cannot create a 4x4 transformation matrix without 4 number arrays of length 4."
        // for (let i = 0; i < 4; i++) {
        //     if (values[i].length != 4)
        //         throw "RangeError: Cannot create a 4x4 transformation matrix without 4 number arrays of length 4."
        //     for (let j = 0; j < 4; j++) {
        //         internalData[i][j] = values[i][j];
        //     }
        // }

        return new TransformationMatrix(matrix.values);
    }

    /** Given a 3x3 matrix, adds a fourth row and col full of 0s, with a 1 in the corner. */
    public static extend3x3Matrix(matrix: Matrix): TransformationMatrix {
        let m33 = matrix.values;
        return new TransformationMatrix([
            m33[0][0], m33[0][1], m33[0][2], 0,
            m33[1][0], m33[1][1], m33[1][2], 0,
            m33[2][0], m33[2][1], m33[2][2], 0,
                    0,         0,         0, 1
        ])
    }

    /** Return a copy of this transformation matrix */
    public clone(): TransformationMatrix {
        return new TransformationMatrix(this.values);
    }

    /** Copy the values of an other transformation matrix. Overrides the data of this matrix. */
    public copy(other: TransformationMatrix) {
        this.values = other.values;
    }

    /** Create a translation matrix from x y and z. */
    public static translationMatrix(x: number, y: number, z: number): TransformationMatrix;
    /** Create a translation matrix from x y and z. Array format should be [x, y, z].*/
    public static translationMatrix(data: number[]): TransformationMatrix;
    /** Create a translation matrix from x and y. Assumes z translation of zero. */
    public static translationMatrix(data: Vector2): TransformationMatrix;
    /** Create a translation matrix from x y and z. */
    public static translationMatrix(data: Vector3): TransformationMatrix;
    public static translationMatrix(data: number[] | Vector): TransformationMatrix;
    public static translationMatrix(param1: Vector | number[] | number, param2?: number, param3?: number): TransformationMatrix {
        let tx: number;
        let ty: number;
        let tz: number;
        if (typeof param1 == "number") {
            tx = param1;
            ty = param2;
            tz = param3;
        } else if (param1 instanceof Vector) {
            tx = param1.x;
            ty = param1.y;
            tz = param1.z;
        } else {
            tx = param1[0];
            ty = param1[1];
            tz = param1[2];
        }

        return new TransformationMatrix([
            [1, 0, 0, tx],
            [0, 1, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0, 1],
        ]);
    }

    /** Create a rotation matrix from x y and z rotation. 3-2-1 rotation order. */
    public static rotationMatrix(x: number, y: number, z: number): TransformationMatrix;
    /** Create a rotation matrix from a quaternion. */
    public static rotationMatrix(data: Quaternion): TransformationMatrix;
    /** Create a rotation matrix from x y rotation. Assumes no rotation for z. 3-2-1 rotation order.  */
    public static rotationMatrix(data: Vector2): TransformationMatrix;
    /** Create a rotation matrix from x y and z rotation. 3-2-1 rotation order. */
    public static rotationMatrix(data: Vector3): TransformationMatrix;
    /** Create a rotation matrix from x y and z rotation. 3-2-1 rotation order. Array format should be [x, y, z]. */
    public static rotationMatrix(data: number[]): TransformationMatrix;
    public static rotationMatrix(data: Quaternion | Vector | number[]): TransformationMatrix;
    public static rotationMatrix(param1: Quaternion | Vector | number[] | number, param2?: number, param3?: number): TransformationMatrix {
        if (param1 instanceof Quaternion) {
            return TransformationMatrix.fromMatrix(param1.toRotationMatrix4x4());
        } else if (param1 instanceof Vector) {
            if (param1 instanceof Vector2) {
                return TransformationMatrix.fromMatrix(Quaternion.fromEulerAngles(param1.toVector3()).toRotationMatrix4x4());
            }
            if (param1 instanceof Vector3) {
                return TransformationMatrix.fromMatrix(Quaternion.fromEulerAngles(param1).toRotationMatrix4x4());
            }
        } else if (typeof param1 == "number") {
            return TransformationMatrix.fromMatrix(Quaternion.fromEulerAngles(param1, param2, param3).toRotationMatrix4x4());
        } else {
            return TransformationMatrix.fromMatrix(Quaternion.fromEulerAngles(param1[0], param1[1], param1[2]).toRotationMatrix4x4());
        }
        return TransformationMatrix.Identity();
    }

    /** Generate a scale matrix with different values for each axis. */
    public static scaleMatrix(x: number, y: number, z: number): TransformationMatrix;
    /** Generate a uniform scale matrix. Scaling is the same on each axis. */
    public static scaleMatrix(scale: number): TransformationMatrix;
    /** Generate a scale matrix with different values for each axis. Array format should be [x, y, z]. */
    public static scaleMatrix(data: number[]): TransformationMatrix;
    /** Generate a scale matrix with different values for the x and y axis. Assumes a scale on the z axis of one. */
    public static scaleMatrix(data: Vector2): TransformationMatrix;
    /** Generate a scale matrix with different values for each axis. */
    public static scaleMatrix(data: Vector3): TransformationMatrix;
    public static scaleMatrix(data: Vector | number): TransformationMatrix;
    public static scaleMatrix(param1: number | Vector | number[], param2?: number, param3?: number): TransformationMatrix {
        let sx = 1;
        let sy = 1;
        let sz = 1;
        if (param1 instanceof Vector) {
            sx = param1.x;
            sy = param1.y;
            if (param1 instanceof Vector3) {
                sz = param1.z;
            }
        } else if (Array.isArray(param1)) {
            sx = (param1 as number[])[0];
            sy = (param1 as number[])[1];
            sz = (param1 as number[])[2];
        } else {
            sx = sy = sz = param1 as number;
            if (param2 !== undefined) {
                sy = param2;
                sz = param3;
            }
        }

        return new TransformationMatrix([
            [sx, 0, 0, 0],
            [0, sy, 0, 0],
            [0, 0, sz, 0],
            [0, 0, 0, 1],
        ])
    }

    /** 
     * Completely untested skew function. 
     * Takes the coefficients for skewing the x axis with respect to the y and z axes, then the y axis with respect to the x and z axes, then z with respect to the x and y axes. 
     * When to apply or how to use? Probably before or after scale.
     */
    public static skewMatrix(XY: number, XZ: number, YX: number, YZ: number, ZX: number, ZY: number) {
        return new TransformationMatrix([
            [1, XY, XZ, 0],
            [YX, 1, YZ, 0],
            [ZX, ZY, 1, 0],
            [0, 0, 0, 1],
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
        return new TransformationMatrix([
            [1 - 2 * a * a, -2 * a * b, -2 * a * c, -2 * a * d],
            [-2 * a * b, 1 - 2 * b * b, -2 * b * c, -2 * b * d],
            [-2 * a * c, -2 * b * c, 1 - 2 * c * c, -2 * c * d],
            [0, 0, 0, 1],
        ]);
    }

    /** Create a full transformation matrix from a translation, a rotation, and a scale. Standard arguments for a 2D object. Rotation angle is applied to the Z axis.*/
    public static constructTranslationRotationScaleMatrix(translation: Vector2, rotation: number, scale: Vector2): TransformationMatrix;
    /** Create a full transformation matrix from a translation, a rotation, and a scale. Standard arguments for a 3D object.*/
    public static constructTranslationRotationScaleMatrix(translation: Vector3, rotation: Quaternion, scale: Vector3): TransformationMatrix;
    /** Create a full transformation matrix from a translation, a rotation, and a scale. */
    public static constructTranslationRotationScaleMatrix(_translation: TransformationMatrix | Matrix | Vector | number[], _rotation: Matrix | Quaternion | Vector | number[] | number, _scale: Matrix | Vector | number): TransformationMatrix {
        let translation: TransformationMatrix;
        let rotation: TransformationMatrix;
        let scale: TransformationMatrix;

        if (_translation instanceof TransformationMatrix) {
            translation = _translation;
        } else if (_translation instanceof Matrix) {
            translation = TransformationMatrix.fromMatrix(_translation);
        } else {
            translation = TransformationMatrix.translationMatrix(_translation);
        }

        if (_rotation instanceof TransformationMatrix) {
            rotation = _rotation;
        } else if (_rotation instanceof Matrix) {
            rotation = TransformationMatrix.fromMatrix(_rotation);
        } else if (typeof _rotation == "number") {
            rotation = TransformationMatrix.rotationMatrix(0, 0, _rotation);
        } else {
            rotation = TransformationMatrix.rotationMatrix(_rotation);
        }

        if (_scale instanceof TransformationMatrix) {
            scale = _scale;
        } else if (_scale instanceof Matrix) {
            scale = TransformationMatrix.fromMatrix(_scale);
        } else {
            scale = TransformationMatrix.scaleMatrix(_scale);
        }

        // scale the object "locally", then rotate it "locally", then translate it "globally"
        let identity = TransformationMatrix.Identity();
        return identity.thenApply(scale).thenApply(rotation).thenApply(translation);
    }

    /** 
     * Apply this transformation matrix to a Vector3.
     * Faster than converting the Vector3 to a 1x4 matrix and doing matrix multiplication.
     */
    public applyToVector3(vector: Vector3): Vector3 {
        let resultArr: number[] = [0, 0, 0, 0];
        for (let c = 0; c < 4; ++c) {
            resultArr[0] += this.values[0][c] * vector.x;
            resultArr[1] += this.values[1][c] * vector.y;
            resultArr[2] += this.values[2][c] * vector.z;
            //resultArr[3] += lhs.values[3][c] * 1;
        }
        return new Vector3(resultArr[0], resultArr[1], resultArr[2]);
    }

    /** of Matrix.multiply that specifically returns a TransformationMatrix */
    public static multiply(lhs: TransformationMatrix, rhs: TransformationMatrix): TransformationMatrix {


        let m: internalData4x4;  // declare internal data structure
        for (let r = 0; r < 4; ++r) {
            m[r] = [0, 0, 0, 0]; // initialize the current row
            for (let c = 0; c < 4; ++c) {
                m[r][c] = 0; // initialize the current cell
                for (let i = 0; i < 4; ++i) {
                    m[r][c] += lhs.values[r][i] * rhs.values[i][c];
                }
            }
        }
        return new TransformationMatrix(m);
    }



    /** Return a new transformation matrix corresponding to this transformation, then an other transformation. */
    public thenApply(other: TransformationMatrix): TransformationMatrix {
        return TransformationMatrix.multiply(this, other);
    }

    /** Return a new transformation matrix corresponding to this an other transformation, then this transformation. */
    // #FIXME necessary? when would this ever be used? also confusing with thenApply.
    public applyThen(other: TransformationMatrix): TransformationMatrix {
        return TransformationMatrix.multiply(other, this);
    }

    // #TODO create functions that alter this matrix instead of returning a new one
}