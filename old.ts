

namespace MathVQ {

    const buf = Buffer.create(4);
    const threehalfs: number = 1.5;
    export function fastInvSqrRoot(num: number) {
        let i;
        let x2: number, y: number;

        x2 = num >> 1;
        y = num;
        //evil floating point bit level hacking replacement
        buf.setNumber(NumberFormat.Float32LE, 0, num);
        i = buf.getNumber(NumberFormat.UInt32LE, 0);
        i = (0x5f3759df - (i >> 1)); //What the heck?
        buf.setNumber(NumberFormat.UInt32LE, 0, i);
        y = buf.getNumber(NumberFormat.Float32LE, 0);
        y = y * (threehalfs - (x2 * y * y));  // 1st iteration
        // y = y * (threehalfs - (x2 * y * y));  // 2nd iteration, this can be removed
        
        return y;
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

        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    /** 
     * Generic distance function taking any combination of Vector2s and Vector3s. 
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
        let doublemagsqr = lhs.sqrMagnitude() * rhs.sqrMagnitude();
        return dot(lhs, rhs) / Math.sqrt(doublemagsqr);
    }
}

// abstract class Vector {
    public x: number;
    public y: number;
    public z: number;

    public constructor() {}

/**
 * 
 */
    // all four of these are abstract, but without keyword
    public toString(): string { return ""; }
    public toArray(): number[] { return []; }
    // 'public static fromArray(values: number[]): type' subclasses should have something like this
    public clone(): Vector { return null; }
    public copy(other: Vector): void { }

    // Remember to put conversion functions too

    /*
     * These 6 functions are done here generically, since they should work with any vector.
     * sqrMagnitude is overridden in Vector2 to save on multiplications.
     * Technically a lot of these would be faster if done specifically between two Vector2s,
     * but that would need 'instanceOf(Vector2)' which would be slower.
     * Consider adding special 'distToVec2' type functions #TODO
     */

    /** Returns the distance to another vector. */
    public sqrDistanceTo(other: Vector): number {
        return MathVQ.sqrDistance(this, other);
    }

    /** Returns the distance to another vector. */
    public distanceTo(other: Vector): number {
        return MathVQ.distance(this, other);
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
    public dotWith(other: Vector): number {
        return MathVQ.dot(this, other);
    }

    /** Dot product another vector with this. */
    public withDot(other: Vector): number {
        return MathVQ.dot(other, this);
    }

    // All abstract, but without keyword
    public normalised(): Vector { return null; }
    public normalise(): void { }
    public add(other: Vector): Vector { return null; }
    public addSelf(other: Vector): void { }
    public sub(other: Vector): Vector { return null; }
    public subSelf(other: Vector): void { }
    public mult(other: number): Vector { return null; }
    public multSelf(other: number): void { }
    public div(other: number): Vector { return null; }
    public divSelf(other: number): void { }
    // #FIXME scaled and scale to match normalisation? scaledBy and scaleBy? (that one seems silly) scale and scaleSelf to match other operations?
    public scaledBy(other: Vector): Vector { return null; }
    public scaleBy(other: Vector): void { }
}

class Vector31 extends Vector {

    public constructor(x: number, y: number, z: number) {
        super();
        this.x = x;
        this.y = y;
        this.z = z;
    }

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

    /** Convert to string. */
    public toString(): string {
        return "(" + this.x + ", " + this.y + ", " + this.z + ")";
    }

    /** Convert to array of [x, y, z]. */
    public toArray(): number[] {
        return [this.x, this.y, this.z];
    }

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

    /** Copy the values of another Vector3 to this. Overrides this Vector3s values. */
    public copy(other: Vector3): void {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    /** Create a new Vector2 with the same x and y, dropping the z.*/
    public toVector2(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    public normalised(): Vector { 
        return this.div(this.magnitude());
    }
    public normalise(): void { 

    }
    public add(other: Vector): Vector {
        return null;
    }
    public addSelf(other: Vector): void {

     }
    public sub(other: Vector): Vector {
        return null;
    }
    public subSelf(other: Vector): void { 

    }
    public mult(other: number): Vector {
        return null;
    }
    public multSelf(other: number): void { 

    }
    public div(other: number): Vector {
        return null;
    }
    public divSelf(other: number): void { 

    }
    public scaledBy(other: Vector): Vector {
        return null;
    }
    public scaleBy(other: Vector): void {

    }

}

class Vector22 extends Vector {

    public get z() {
        return 0;
    }
    public set z(nz: number) {
        throw ("not allowed to set z of a vec2");
    }

    public constructor(x: number, y: number) {
        super();
        this.x = x;
        this.y = y;
    }

    /** Returns the vector (1, 1). */
    public static One(): Vector2 {
        return new Vector2(1, 1);
    }
    /** Returns the vector (0, 0). */
    public static Zero(): Vector2 {
        return new Vector2(0, 0);
    }
    /** Returns the vector (-1, 0). */
    public static Left(): Vector2 {
        return new Vector2(-1, 0);
    }
    /** Returns the vector (1, 0). */
    public static Right(): Vector2 {
        return new Vector2(1, 0);
    }
    /** Returns the vector (0, 1). */
    public static Up(): Vector2 {
        return new Vector2(0, 1);
    }
    /** Returns the vector (0, -1). */
    public static Down(): Vector2 {
        return new Vector2(0, -1);
    }

    /** Convert to string. */
    public toString(): string {
        return "(" + this.x + ", " + this.y + ")";
    }

    /** Convert to array of [x, y, z]. */
    public toArray(): number[] {
        return [this.x, this.y, this.z];
    }

    /** Convert a 3 element array to a Vector3 */
    public static fromArray(values: number[]): Vector2 {
        if (values.length < 2)
            throw "RangeError: Cannot create a Vector2 from an array with less than 2 elements.";
        return new Vector2(values[0], values[1]);
    }

    /** Return a copy of this Vector3. */
    public clone(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    /** Copy the values of another Vector3 to this. Overrides this Vector3s values. */
    public copy(other: Vector2): void {
        this.x = other.x;
        this.y = other.y;
    }

    /** Create a new Vector3 with the same x and y, and a new z. Leave empty to set z to 0. */
    public toVector3(z?: number): Vector3 {
        if (z)
            return new Vector3(this.x, this.y, z);
        return new Vector3(this.x, this.y, 0);
    }

    // This one is overridden for Vector2 to save on multiplications.
    /** 
     * Gets the magnitude/length of this vector squared. 
     * Faster than Magnitude() ** 2. 
    */
    public sqrMagnitude(): number {
        return (this.x * this.x) + (this.y * this.y);
    }

    public normalised(): Vector {
        return null;
    }
    public normalise(): void {

    }
    public add(other: Vector): Vector {
        return null;
    }
    public addSelf(other: Vector): void {

    }
    public sub(other: Vector): Vector {
        return null;
    }
    public subSelf(other: Vector): void {

    }
    public mult(other: number): Vector {
        return null;
    }
    public multSelf(other: number): void {

    }
    public div(other: number): Vector {
        return null;
    }
    public divSelf(other: number): void {

    }
    public scaledBy(other: Vector): Vector {
        return null;
    }
    public scaleBy(other: Vector): void {

    }
}
