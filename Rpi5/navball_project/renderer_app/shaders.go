package main

// Shader สำหรับภาพกล้องพื้นหลัง
const vertexShaderSource = `
attribute vec2 aPos;
attribute vec2 aTexCoord;
varying vec2 TexCoord;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
    TexCoord = aTexCoord;
}
`

const fragmentShaderSource = `
varying vec2 TexCoord;
uniform sampler2D uTexture;
void main() {
    gl_FragColor = texture2D(uTexture, TexCoord);
}
`

// --- Shader สำหรับวาด Navball (วงกลมขอบฟ้าจำลอง) ---
const navballVertexShader = `
attribute vec2 aPos;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
}
`

const navballFragmentShader = `
uniform vec2 uCenter;
uniform float uRadius;
uniform float uPitch; // เรเดียน
uniform float uRoll;  // เรเดียน
uniform float uYaw;   // เรเดียน
uniform sampler2D uNavballTexture; // ภาพ navball_brownblue.png (equirectangular 1024x512)

// ระยะจากจุด p ถึงส่วนของเส้นตรง a-b หนา thickness ใช้วาด craft marker
// (เป็น fixed screen-space overlay เท่านั้น ไม่เกี่ยวกับลายบนทรงกลม)
float sdfSegment(vec2 p, vec2 a, vec2 b, float thickness) {
    vec2 pa = p - a;
    vec2 ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    float d = length(pa - ba * h);
    return 1.0 - smoothstep(thickness * 0.5, thickness * 0.5 + 0.015, d);
}

// หมุนรอบแกน X (pitch)
mat3 rotX(float a) {
    float c = cos(a), s = sin(a);
    return mat3(
        1.0, 0.0, 0.0,
        0.0, c, -s,
        0.0, s, c
    );
}

// หมุนรอบแกน Y (yaw)
mat3 rotY(float a) {
    float c = cos(a), s = sin(a);
    return mat3(
        c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c
    );
}

// หมุนรอบแกน Z (roll)
mat3 rotZ(float a) {
    float c = cos(a), s = sin(a);
    return mat3(
        c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0
    );
}

void main() {
    // พิกัด screen-space เทียบจุดศูนย์กลาง navball, ปรับสเกลด้วยรัศมี
    vec2 p = (gl_FragCoord.xy - uCenter) / uRadius;
    float distXY = length(p);

    // นอกวงกลม -> โปร่งใส
    if (distXY > 1.0) {
        discard;
    }

    // ยิงเรย์ orthographic เข้าไปในทรงกลมหน่วย เพื่อหาจุดตัดบนผิวทรงกลม
    // (มองจากแกน +Z เข้าหา, จุดตัดที่ใกล้กล้องอยู่ด้าน +Z)
    float zSq = 1.0 - distXY * distXY;
    float z = sqrt(max(zSq, 0.0));
    vec3 surfacePoint = vec3(p.x, p.y, z);

    // หมุนทรงกลม "กลับทาง" ด้วย pitch/roll/yaw เพื่อจำลองมุมของยานจริง
    // ลำดับ: Roll (Z) -> Pitch (X) -> Yaw (Y) ใช้ inverse rotation
    // (ลบมุม + ลำดับย้อนกลับ) กับจุดบนผิวทรงกลมที่มองเห็น
    mat3 invRot = rotY(-uYaw) * rotX(-uPitch) * rotZ(-uRoll);
    vec3 worldPoint = invRot * surfacePoint;

    // -------------------------------------------------------------
    // Texture mapping: sample สีจาก navball_brownblue.png (equirectangular)
    // แทนการคำนวณสีท้องฟ้า/พื้นดิน/เส้น ladder/heading ด้วยโค้ดแบบเดิม
    // สูตรมาตรฐาน equirectangular:
    //   u = atan(x, z) / (2*pi) + 0.5   (ลองจิจูดรอบแกน Y, 0..1)
    //   v = asin(y) / pi + 0.5          (ละติจูดจากขั้วล่างถึงขั้วบน, 0..1)
    // -------------------------------------------------------------
    vec2 uv;
    uv.x = atan(worldPoint.x, worldPoint.z) / (2.0 * 3.14159265359) + 0.5;
    uv.y = asin(clamp(worldPoint.y, -1.0, 1.0)) / 3.14159265359 + 0.5;
    gl_FragColor = texture2D(uNavballTexture, uv);

    // ขอบวงแหวนบางๆ รอบตัวลูกบอล กันขอบดูแตกเป็นแฉก
    if (distXY > 0.985) {
        gl_FragColor.a *= (1.0 - distXY) / 0.015;
    }

    // -------------------------------------------------------------
    // Craft Marker / Boresight Marker (รูปตัว V คว่ำ): fixed อยู่กึ่งกลาง
    // จอ Navball เสมอ ไม่หมุนตาม pitch/roll/yaw ของยาน เพราะเป็นตัวบอก
    // "แนวหัวยานชี้ตรงไหนบนจอ" ไม่ใช่ส่วนหนึ่งของพื้นผิวทรงกลม จึงวาด
    // ทับหลังสุด โดยอิง p (พิกัด screen-space เทียบจุดศูนย์กลาง) ตรงๆ
    // ไม่ผ่าน rotation ใดๆ
    // -------------------------------------------------------------
    {
        float markerScale = 0.22;
        vec2 mp = p / markerScale; // พิกัด local รอบศูนย์กลางจอ
        // แขนซ้าย: จาก (0,0) ไป (-1,-0.7)
        float armL = sdfSegment(mp, vec2(0.0, 0.05), vec2(-0.9, -0.65), 0.16);
        // แขนขวา: จาก (0,0) ไป (1,-0.7)
        float armR = sdfSegment(mp, vec2(0.0, 0.05), vec2(0.9, -0.65), 0.16);
        float markerCov = max(armL, armR);
        if (markerCov > 0.0) {
            vec4 markerColor = vec4(0.1, 1.0, 0.2, 1.0); // เขียวสดตัดกับพื้นหลัง
            gl_FragColor = mix(gl_FragColor, markerColor, markerCov);
        }
    }
}
`