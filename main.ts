//Vector Quaternion Matrix Extension
namespace VQME {

    export function RotateVec(lhs: Quaternion, rhs: Vec3) {
        let qsame = new Quaternion(lhs.w, lhs.x, lhs.y, lhs.z);
        let qvec = new Quaternion(0, rhs.x, rhs.y, rhs.z);
        //let qinv = qsame.Conjugate();
        let qinv = new Quaternion(qsame.w, -qsame.x, -qsame.y, -qsame.z);

        let qoutq = VQME.RotateQ(qsame, VQME.RotateQ(qvec, qinv));
        return new Vec3(qoutq.x, qoutq.y, qoutq.z);
    }

    export function Dot3(lhs: Vec3, rhs: Vec3) {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    export function Cross3(lhs: Vec3, rhs: Vec3) {
        let nx = lhs.y * rhs.z - lhs.z * rhs.y;
        let ny = lhs.z * rhs.x - lhs.x * rhs.z;
        let nz = lhs.x * rhs.y - lhs.y * rhs.x;
        return new Vec3(nx, ny, nz);
    }

    export function RotateQ(lhs: Quaternion, rhs: Quaternion) {
        let a = lhs;
        let b = rhs;

        let nqw = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        let nqx = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        let nqy = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        let nqz = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

        return new VQME.Quaternion(nqw, nqx, nqy, nqz);
    }

    //add two vectors together
    export function Add3(lhs: Vec3, rhs: Vec3) {
        return new Vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    //subtract vector from a vector
    export function Subtract3(lhs: Vec3, rhs: Vec3) {
        return new Vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    //multiply a vector by a scalar
    export function Multiply3(v: Vec3, s: number) {
        return new Vec3(v.x * s, v.y * s, v.z * s);
    }

    //divide a vector by a scalar
    export function Divide3(v: Vec3, s: number) {
        return new Vec3(v.x / s, v.y / s, v.z / s);
    }

    export function Multiply2(v: Vec2, s: number) {
        return new Vec2(v.x * s, v.y * s);
    }

    //find the distance between two vectors squared (faster than Distance() ** 2)
    export function SqrDistance3(posA: Vec3, posB: Vec3) {
        return ((posA.x - posB.x) ** 2) + ((posA.y - posB.y) ** 2) + ((posA.z - posB.z) ** 2);
    }


    //find the distance between two vectors squared (faster than Distance() ** 2)
    export function SqrDistance2(posA: Vec2, posB: Vec2) {
        return ((posA.x - posB.x) ** 2) + ((posA.y - posB.y) ** 2);
    }

    //find the distance between two vectors
    export function Distance3(posA: Vec3, posB: Vec3) {
        return Math.sqrt(SqrDistance3(posA, posB));
    }

    //find the distance between two vectors
    export function Distance2(posA: Vec2, posB: Vec2) {
        return Math.sqrt(SqrDistance2(posA, posB));
    }

    //get the dot product of two vectors divided by both their magnitudes
    export function Dot3Norm(lhs: Vec3, rhs: Vec3) {
        let doublemag = lhs.Magnitude() * rhs.Magnitude();
        return (Dot3(lhs, rhs) / doublemag);
    }

    //returns the angle between two vectors
    export function Angle(lhs: Vec3, rhs: Vec3) {
        return Math.acos(Dot3Norm(lhs, rhs));
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

        static One = new Vec3(1, 1, 1);
        static Zero = new Vec3(0, 0, 0);

        //convert to string
        ToString() {
            return "(" + this.x + ", " + this.y + ", " + this.z + ")"
        }

        //conver this to an array
        ToArray() {
            return [this.x, this.y, this.z];
        }

        DistanceTo(other: Vec3) {
            return Distance3(this, other);
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
            return Multiply3(this, scale);
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
        PlusEquals(pos: Vec3) {
            this.x += pos.x;
            this.y += pos.y;
            this.z += pos.z;
        }

        //return a copy of this vector with another vector added on
        Plus(pos: Vec3) {
            return new Vec3(this.x + pos.x, this.y + pos.y, this.z + pos.z);
        }

        //add a vector to this vector
        MinusEquals(pos: Vec3) {
            this.x -= pos.x;
            this.y -= pos.y;
            this.z -= pos.z;
        }

        //return a copy of this vector with another vector added on
        Minus(pos: Vec3) {
            return new Vec3(this.x - pos.x, this.y - pos.y, this.z - pos.z);
        }

        DotWith(other: Vec3) {
            return Dot3(this, other);
        }

        WithDot(other: Vec3) {
            return Dot3(other, this);
        }

        FormatToV4Matrix() {
            return new Matrix([[this.x], [this.y], [this.z], [1]]);
        }
    }
    export class Vec2 {
        x: number;
        y: number;
        z: number;

        constructor(x: number, y: number) {
            this.x = x;
            this.y = y;
        }

        static One = new Vec2(1, 1);
        static Zero = new Vec2(0, 0);

        //convert to string
        ToString() {
            return "(" + this.x + ", " + this.y + ")";
        }

        //conver this to an array
        ToArray() {
            return [this.x, this.y];
        }

        DistanceTo(other: Vec2) {
            return Distance2(this, other);
        }

        //gets the magnitude/length of this vector
        Magnitude() {
            return Math.sqrt(this.SqrMagnitude());
        }

        //gets the magnitude/length of this vector squared. faster than Magnitude() ** 2
        SqrMagnitude() {
            return (this.x ** 2) + (this.y ** 2);
        }

        //normalise this vector (magnitude/length of 1)
        Normalise() {
            let mag = this.Magnitude();
            this.x /= mag;
            this.y /= mag;
        }

        //return a normalised copy of this vector (magnitude/length of 1)
        Normalised() {
            let mag = this.Magnitude();
            return new Vec2(this.x / mag, this.y / mag);
        }


        //return a copy of this vector scaled by a number
        Times(scale: number) {
            return Multiply2(this, scale);
        }

        //return a copy of this vector scaled differently on x y and z
        VTimes(scale: Vec2) {
            return new Vec2(this.x * scale.x, this.y * scale.y);
        }

        //scale this vector by a number
        TimesEquals(scale: number) {
            this.x *= scale;
            this.y *= scale;
            this.z *= scale;
        }

        //scale this vector differently on x y and z
        VTimesEquals(scale: Vec2) {
            this.x *= scale.x;
            this.y *= scale.y;
            this.z *= scale.z;
        }

        //add a vector to this vector
        PlusEquals(pos: Vec2) {
            this.x += pos.x;
            this.y += pos.y;
            this.z += pos.z;
        }

        //return a copy of this vector with another vector added on
        Plus(pos: Vec2) {
            return new Vec2(this.x + pos.x, this.y + pos.y);
        }

        //add a vector to this vector
        MinusEquals(pos: Vec2) {
            this.x -= pos.x;
            this.y -= pos.y;
            this.z -= pos.z;
        }

        //return a copy of this vector with another vector added on
        Minus(pos: Vec2) {
            return new Vec2(this.x - pos.x, this.y - pos.y);
        }

        ToVec3(z: number) {
            return new Vec3(this.x, this.y, z);
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

        //rotate lhs by rhs
        RotateFirst(other: Quaternion) {
            return RotateQ(other, this);
        }

        RotateSecond(other: Quaternion) {
            return RotateQ(this, other);
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
        static FromEulerAnglesVec3(vec: Vec3) {
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

        Normalised() {
            let q = new Quaternion(this.w, this.x, this.y, this.z);
            let mag = q.Magnitude();
            q.w /= mag;
            q.x /= mag;
            q.y /= mag;
            q.z /= mag;
            return q;
        }

        Normalise() {
            let mag = this.Magnitude();
            this.w /= mag;
            this.x /= mag;
            this.y /= mag;
            this.z /= mag;
        }

        Conjugate() {
            return new Quaternion(this.w, -this.x, -this.y, -this.z);
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

        ToEulerAngles() {
            return Quaternion.ToEulerAngles(this);
        }

        ToArray() {
            return [this.w, this.x, this.y, this.z];
        }

        //untested
        ToRotationMatrix() {
            let q = this.ToArray(); //wxyz? xyzw?
            //let q = [qte[1], qte[2], qte[3], qte[0]];
            return new Matrix([
                [2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])],
                [2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1])],
                [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1],
            ]);
        }

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
}

namespace DFPoly {

    //cull vertices based on clip space
    export function ClipspaceCull(info: MeshInfo, cam: Camera) {
        let newTrigs: number[] = [];
        let nind = 0;

        for(let i = 0; i < info.triangles.length; i += 3) {
            let currentTrig = info.GetTrigAtIndex(i);
            if (!BackfaceCull(currentTrig, cam)) {
                newTrigs.length += 3;
                newTrigs[nind + 0] = info.triangles[i + 0];
                newTrigs[nind + 1] = info.triangles[i + 1];
                newTrigs[nind + 2] = info.triangles[i + 2];
                nind += 3;
            }
        }
        
        return newTrigs;
    }

    //implement with edge cases later
    export function ClipPoly(trig: VQME.Vec3[], cam: Camera) {
        let output = { rejected: false, outTrigs: [new VQME.Vec3(0, 0, 0)] }
        let outCount = 0;
        let outInds = 0;
        for(let i = 0; i <3 ; i++) {
            let curr = trig[i];
            if (curr.z > cam.zFar || curr.z < cam.zNear) {
                outCount += 1;
                outInds |= 1 << i;
            } else if (curr.x > 1 || curr.x < 0 || curr.y > 1 || curr.y < 0) {
                outCount += 1;
                outInds |= 1 << i;
            }
        }
        //trivial accept and trivial reject
        if (outCount == 0) {
            output.outTrigs = trig;
        }
        if (outCount == 3) {
            output.rejected = true;
        }
        //for 1 and 2 consider edge cases
        if(outCount == 2) {
        
        }
        return output;
    }

    export function CalcTrigNormal(trig: VQME.Vec3[]) {
        let U = trig[1].Minus(trig[0]);
        let V = trig[2].Minus(trig[0]);

        let nx = (U.y * V.z) - (U.z * V.y);
        let ny = (U.z * V.x) - (U.x * V.z);
        let nz = (U.x * V.y) - (U.y * V.x);

        return new VQME.Vec3(nx, ny, nz);
    }

    //rejected is true if the forward and normal are same direction (>0)
    export function BackfaceCull(trig: VQME.Vec3[], cam: Camera) {
        return VQME.Dot3(cam.ForwardVec(), CalcTrigNormal(trig)) > 0;
    }

    export class Camera {
        position: VQME.Vec3;
        rotation: VQME.Quaternion;
        fov: number;
        aspectRatio: number;
        zNear: number;
        zFar: number;
        screenWidth: number;
        screenHeight: number;

        ViewMatrix() {
            return new VQME.Matrix([
                [1, 0, 0, -this.position.x],
                [0, 1, 0, -this.position.y],
                [0, 0, 1, -this.position.z],
                [0, 0, 0, 1],
            ])
        }

        RotationMatrix() {
            let inv = new VQME.Quaternion(this.rotation.w, -this.rotation.x, -this.rotation.y, -this.rotation.z);
            return inv.ToRotationMatrix4();
        }

        ForwardVec() {
            return VQME.RotateVec(this.rotation, new VQME.Vec3(0, 0, 1))
        }

        PerspectiveProjectionMatrix() {
            let invtanfov = 1 / Math.tan(this.fov / 2);
            let invasp = invtanfov / this.aspectRatio;

            let zdiff = (this.zNear - this.zFar);
            let m33 = (this.zFar + this.zNear) / zdiff;
            let m43 = (this.zNear * this.zFar * 2) / zdiff;

            return new VQME.Matrix([
                [invasp, 0, 0, 0],
                [0, invtanfov, 0, 0],
                [0, 0, m33, -1],
                [0, 0, m43, 0]
            ]);
        }

        ClipToScreenPoint(point: VQME.Matrix) {
            let divisor = point.values[3][0] * 2;
            let screenX = (point.values[0][0] + 1) * this.screenWidth / divisor;
            let screenY = (-point.values[1][0] + 1) * this.screenHeight / divisor;
            return new VQME.Vec3(screenX, screenY, 0); //make vec2 class
        }
    }

    export class MeshInfo {
        vertices: VQME.Vec3[];
        triangles: number[];
        uvs: VQME.Vec2[];
        texture: number[];

        constructor(vertices: VQME.Vec3[], triangles: number[], uvs: VQME.Vec2[], texture: number[]) {
            this.vertices = vertices;
            this.triangles = triangles;
            this.uvs = uvs;
            this.texture = texture;
        }

        static Copy(mesh: MeshInfo) {
            return new MeshInfo(mesh.vertices, mesh.triangles, mesh.uvs, mesh.texture)
        }

        GetTrigAtIndex(index: number){
            return [this.vertices[this.triangles[index]], this.vertices[this.triangles[index + 1]], this.vertices[this.triangles[index + 2]]];
        }
    }

    export class Ob3 {
        position: VQME.Vec3;
        rotation: VQME.Quaternion;

        info: MeshInfo;
        
        rotatedVerts: VQME.Vec3[];
        transformedVerts: VQME.Vec3[];

        boundingMin: VQME.Vec3;
        boundingMax: VQME.Vec3;

        constructor(position: VQME.Vec3, rotation: VQME.Quaternion, info: MeshInfo) {
            this.position = position;
            this.rotation = rotation;
            
            this.info = info;

            this.rotatedVerts = [];
            this.transformedVerts = [];

            this.rotatedVerts.length = info.vertices.length;
            this.transformedVerts.length = info.vertices.length;

            this.UpdateTransformedVecs();
        }

        Rotate(change: VQME.Quaternion) {
            this.rotation = VQME.RotateQ(this.rotation, change);
            this.UpdateTransformedVecs();
        }

        Translate(change: VQME.Vec3) {
            this.position.PlusEquals(change);
            this.UpdateTransformedVecs();
        }

        UpdateTransformedVecs() {
            for (let i = 0; i < this.info.vertices.length; i++) {
                this.rotatedVerts[i] = VQME.RotateVec(this.rotation, this.info.vertices[i]);
                let tvert = this.rotatedVerts[i].Plus(this.position);
                this.transformedVerts[i] = tvert;
            }
        }

        RecalculateBoundingBox() {
            let minx = 99999;
            let maxx = -99999;
            let miny = 99999;
            let maxy = -99999;
            let minz = 99999;
            let maxz = -99999;

            for(let i = 0; i < this.transformedVerts.length; i++) {
                let tvert = this.transformedVerts[i];
                if (tvert.x > maxx) maxx = tvert.x;
                if (tvert.x < minx) minx = tvert.x;
                if (tvert.y > maxy) maxy = tvert.y;
                if (tvert.y < miny) miny = tvert.y;
                if (tvert.z > maxz) maxz = tvert.z;
                if (tvert.z < minz) minz = tvert.z;
            }

            this.boundingMin = new VQME.Vec3(minx, miny, minz);
            this.boundingMax = new VQME.Vec3(maxx, maxy, maxz);
        }

        static Instantiate(prefab: Ob3, position: VQME.Vec3, rotation: VQME.Quaternion) {
            return 
        }
    }

    export class MeshInfoStorer {
        static meshRegistry: {[key: string] : MeshInfo} = { };

        static AddToDict(key: string, mesh: MeshInfo) {
            MeshInfoStorer.meshRegistry[key] = MeshInfo.Copy(mesh);
        }

        static GetCopyFromDict(key: string) {
            let mesh = MeshInfoStorer.meshRegistry[key];
            return MeshInfo.Copy(mesh);
        }
    }
}