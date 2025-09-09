/*
  ESP32 Landslide Monitor - 3D Viewer
  Updated for BLE connection and real-time 3D visualization
*/

let scene, camera, renderer, cube;
let bleDevice = null;
let bleServer = null;
let bleService = null;
let bleCharacteristic = null;
let isConnected = false;

// Sensor data
let sensorData = {
    elevation: 0.0,
    distance: 0.0,
    roll: 0.0,
    yaw: 0.0,
    mode: 1
};

// Zero offsets for 3D visualization
let zeroOffsets = {
    elevation: 0.0,
    roll: 0.0,
    yaw: 0.0
};

// BLE Configuration (must match ESP32)
const SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const DEVICE_NAME = "ESP32_LANDSLIDE_MOCK";

// DOM elements
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const bleStatus = document.getElementById('bleStatus');
const deviceName = document.getElementById('deviceName');

        // ลบ debug info ออกแล้ว

// Helper function for degree to radian conversion (compatible with old Three.js)
function degToRad(degrees) {
    return degrees * (Math.PI / 180);
}

// ลบฟังก์ชัน updateDebugInfo ออกแล้ว

// Initialize 3D scene
function init3D() {
    console.log('Initializing 3D scene...');
    
    try {
        // Check if Three.js is loaded
        if (typeof THREE === 'undefined') {
            console.error('Three.js not loaded!');
            alert('Three.js library not loaded. Please check your internet connection.');
            return;
        }
        
        console.log('Three.js version:', THREE.REVISION);
        
        // Get container
        const container = document.getElementById("3Dcube");
        if (!container) {
            console.error('3D container not found!');
            return;
        }
        
        // Force container dimensions if they're 0
        if (container.clientWidth === 0 || container.clientHeight === 0) {
            container.style.width = '100%';
            container.style.height = '500px';
            console.log('Forced container dimensions');
        }
        
        console.log('Container dimensions:', container.clientWidth, 'x', container.clientHeight);
        
        // Create scene
  scene = new THREE.Scene();
        scene.background = null; // พื้นหลังโปร่งใสสมบูรณ์
        // ลบ fog ออกเพื่อให้ลูกบาศก์ชัดเจน
        
        // Create camera with better position for larger cube
        const aspect = Math.max(container.clientWidth, 1) / Math.max(container.clientHeight, 1);
        camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
        camera.position.set(25, 20, 35); // ปรับตำแหน่งกล้องให้เหมาะสมกับลูกบาศก์ที่ใหญ่ขึ้น
        camera.lookAt(0, 0, 0);
        
        // Create renderer
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setClearColor(0x000000, 0); // พื้นหลังโปร่งใสสมบูรณ์
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.outputEncoding = THREE.sRGBEncoding;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.2;
        
        // Add renderer to container
        container.appendChild(renderer.domElement);
        console.log('Renderer added to container');
        
        // Create MUCH larger, beautiful cube
        const geometry = new THREE.BoxGeometry(20, 8, 16); // เพิ่มขนาดเป็น 3 เท่า!
        
        // Create beautiful blue materials for all faces
        // Create beautiful 3D cube with proper lighting and shadows
        const materials = [
            new THREE.MeshStandardMaterial({ 
                color: 0x1E3A8A, // Dark Blue
                metalness: 0.1,
                roughness: 0.3,
                transparent: false,
                opacity: 1.0
            }),
            new THREE.MeshStandardMaterial({ 
                color: 0x1E3A8A, // Dark Blue
                metalness: 0.1,
                roughness: 0.3,
                transparent: false,
                opacity: 1.0
            }),
            new THREE.MeshStandardMaterial({ 
                color: 0x1E3A8A, // Dark Blue
                metalness: 0.1,
                roughness: 0.3,
                transparent: false,
                opacity: 1.0
            }),
            new THREE.MeshStandardMaterial({ 
                color: 0x1E3A8A, // Dark Blue
                metalness: 0.1,
                roughness: 0.3,
                transparent: false,
                opacity: 1.0
            }),
            new THREE.MeshStandardMaterial({ 
                color: 0x1E3A8A, // Dark Blue
                metalness: 0.1,
                roughness: 0.3,
                transparent: false,
                opacity: 1.0
            }),
            new THREE.MeshStandardMaterial({ 
                color: 0x1E3A8A, // Dark Blue
                metalness: 0.1,
                roughness: 0.3,
                transparent: false,
                opacity: 1.0
            })
        ];
        
        cube = new THREE.Mesh(geometry, materials);
        cube.position.set(0, 0, 0);
        cube.castShadow = true;
        cube.receiveShadow = true;
        scene.add(cube);
        console.log('HUGE beautiful cube created and added to scene');
        
        // ลบ wireframe ออกแล้ว - เหลือแค่ลูกบาศก์อย่างเดียว
        
        // Add custom axes with labels for better visibility
        const axesLength = 30;
        
        // แกน X (สีแดง) - ซ้าย-ขวา
        const xGeometry = new THREE.CylinderGeometry(0.8, 0.8, axesLength, 8);
        const xMaterial = new THREE.MeshBasicMaterial({ color: 0xFF0000 }); // สีแดงเข้ม
        const xAxis = new THREE.Mesh(xGeometry, xMaterial);
        xAxis.rotation.z = Math.PI / 2;
        xAxis.position.set(axesLength/2, 0, 0);
        scene.add(xAxis);
        
        // แกน Y (สีเขียว) - ขึ้น-ลง
        const yGeometry = new THREE.CylinderGeometry(0.8, 0.8, axesLength, 8);
        const yMaterial = new THREE.MeshBasicMaterial({ color: 0x00FF00 }); // สีเขียวเข้ม
        const yAxis = new THREE.Mesh(yGeometry, yMaterial);
        yAxis.position.set(0, axesLength/2, 0);
        scene.add(yAxis);
        
        // แกน Z (สีน้ำเงิน) - หน้า-หลัง
        const zGeometry = new THREE.CylinderGeometry(0.8, 0.8, axesLength, 8);
        const zMaterial = new THREE.MeshBasicMaterial({ color: 0x0000FF }); // สีน้ำเงินเข้ม
        const zAxis = new THREE.Mesh(zGeometry, zMaterial);
        zAxis.rotation.x = Math.PI / 2;
        zAxis.position.set(0, 0, axesLength/2);
        scene.add(zAxis);
        
        // เพิ่มข้อความบอกแกน
        addAxisLabels();
        
        // Add axes helper with better colors and size (larger)
        const axesHelper = new THREE.AxesHelper(25); // เพิ่มขนาดแกน
        scene.add(axesHelper);
        
        // Add beautiful lighting for 3D effect
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 1.5);
        directionalLight.position.set(15, 25, 15);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 100;
        directionalLight.shadow.camera.left = -25;
        directionalLight.shadow.camera.right = 25;
        directionalLight.shadow.camera.top = 25;
        directionalLight.shadow.camera.bottom = -25;
        scene.add(directionalLight);
        
        // Add fill light for better 3D appearance
        const fillLight = new THREE.DirectionalLight(0x4A90E2, 0.8);
        fillLight.position.set(-15, 10, -15);
        scene.add(fillLight);
        
        // ลบ point lights ออกแล้ว - ให้ลูกบาศก์เด่นชัดขึ้น
        
        // ลบพื้นฐานสีเทาออกแล้ว - เหลือแค่ลูกบาศก์ลอยในอวกาศ
        
        // Add decorative elements
        // ลบ decorative elements ออกแล้ว - เหลือแค่ลูกบาศก์อย่างเดียว
        
        // Initial render
        renderer.render(scene, camera);
        console.log('Initial render completed');
        
        // Start animation loop
        animate();
        
    } catch (error) {
        console.error('Error initializing 3D:', error);
        alert('Error initializing 3D scene: ' + error.message);
    }
}

// Add axis labels using HTML overlay
function addAxisLabels() {
    const container = document.getElementById("3Dcube");
    
    // แกน X (สีแดง) - ซ้าย-ขวา
    const xLabel = document.createElement('div');
    xLabel.textContent = 'X-Axis (ROLL)';
    xLabel.style.cssText = `
        position: absolute;
        left: 60%;
        top: 50%;
        color: #FF0000;
        font-weight: bold;
        font-size: 16px;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.8);
        z-index: 1000;
        pointer-events: none;
    `;
    container.appendChild(xLabel);
    
    // แกน Y (สีเขียว) - ขึ้น-ลง
    const yLabel = document.createElement('div');
    yLabel.textContent = 'Y-Axis (PITCH)';
    yLabel.style.cssText = `
        position: absolute;
        left: 50%;
        top: 20%;
        color: #00FF00;
        font-weight: bold;
        font-size: 16px;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.8);
        z-index: 1000;
        pointer-events: none;
    `;
    container.appendChild(yLabel);
    
    // แกน Z (สีน้ำเงิน) - หน้า-หลัง
    const zLabel = document.createElement('div');
    zLabel.textContent = 'Z-Axis (YAW)';
    zLabel.style.cssText = `
        position: absolute;
        left: 30%;
        top: 50%;
        color: #0000FF;
        font-weight: bold;
        font-size: 16px;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.8);
        z-index: 1000;
        pointer-events: none;
    `;
    container.appendChild(zLabel);
}

// Add decorative elements around the cube
function addDecorativeElements() {
            // Add floating particles (more and larger area)
        const particleCount = 80;
        const particles = new THREE.BufferGeometry();
        const positions = new Float32Array(particleCount * 3);
        const colors = new Float32Array(particleCount * 3);
        
        for (let i = 0; i < particleCount; i++) {
            positions[i * 3] = (Math.random() - 0.5) * 100; // เพิ่มพื้นที่อนุภาค
            positions[i * 3 + 1] = Math.random() * 50; // เพิ่มความสูง
            positions[i * 3 + 2] = (Math.random() - 0.5) * 100; // เพิ่มพื้นที่อนุภาค
        
        colors[i * 3] = Math.random();
        colors[i * 3 + 1] = Math.random();
        colors[i * 3 + 2] = Math.random();
    }
    
    particles.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    particles.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    
    const particleMaterial = new THREE.PointsMaterial({
        size: 0.5,
        vertexColors: true,
        transparent: true,
        opacity: 0.8
    });
    
    const particleSystem = new THREE.Points(particles, particleMaterial);
    scene.add(particleSystem);
    
            // Add floating rings (larger and more)
        for (let i = 0; i < 4; i++) {
            const ringGeometry = new THREE.RingGeometry(25 + i * 5, 25.5 + i * 5, 64); // เพิ่มขนาดวงแหวน
        const ringMaterial = new THREE.MeshBasicMaterial({
            color: new THREE.Color().setHSL(i * 0.25, 0.8, 0.6),
            transparent: true,
            opacity: 0.3,
            side: THREE.DoubleSide
        });
        const ring = new THREE.Mesh(ringGeometry, ringMaterial);
        ring.rotation.x = Math.PI / 2;
        ring.position.y = i * 3;
        scene.add(ring);
    }
    
    // Add floating spheres around the cube
    for (let i = 0; i < 6; i++) {
        const sphereGeometry = new THREE.SphereGeometry(0.5, 16, 16);
        const sphereMaterial = new THREE.MeshBasicMaterial({
            color: new THREE.Color().setHSL(i * 0.17, 0.8, 0.6),
            transparent: true,
            opacity: 0.6
        });
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        
        // Position spheres in a circle around the cube
        const angle = (i / 6) * Math.PI * 2;
        const radius = 35; // เพิ่มระยะห่างจากลูกบาศก์
        sphere.position.set(
            Math.cos(angle) * radius,
            Math.random() * 10 - 5,
            Math.sin(angle) * radius
        );
        
        scene.add(sphere);
    }
}

// Animation loop
function animate() {
    if (!scene || !camera || !renderer) {
        console.log('3D components not ready, skipping animation');
        return;
    }
    
    requestAnimationFrame(animate);
    
    // Update cube rotation based on sensor data
    if (cube) {
        const elevationRad = degToRad(sensorData.elevation - zeroOffsets.elevation);
        const rollRad = degToRad(sensorData.roll - zeroOffsets.roll);
        const yawRad = degToRad(sensorData.yaw - zeroOffsets.yaw);
        
        // Apply rotations
        cube.rotation.x = elevationRad;
        cube.rotation.y = yawRad;
        cube.rotation.z = rollRad;
        
        // Render scene
  renderer.render(scene, camera);
}

    // Update debug info
            // ลบการอัปเดต debug info ออกแล้ว
}

// Handle window resize
function onWindowResize() {
    if (!camera || !renderer) return;
    
    const container = document.getElementById("3Dcube");
    const aspect = container.clientWidth / container.clientHeight;
    
    camera.aspect = aspect;
  camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// BLE Connection Functions
async function connectBLE() {
    try {
        if (!navigator.bluetooth) {
            alert('Bluetooth not supported. Please use Chrome or Edge.');
            return;
        }

        bleStatus.textContent = 'Scanning...';
        bleStatus.style.backgroundColor = '#f39c12';
        bleStatus.style.color = 'white';

        // Request BLE device
        bleDevice = await navigator.bluetooth.requestDevice({
            filters: [
                { name: DEVICE_NAME },
                { namePrefix: 'ESP32' },
                { services: [SERVICE_UUID] }
            ],
            optionalServices: [SERVICE_UUID]
        });

        bleStatus.textContent = 'Connecting...';
        bleStatus.style.backgroundColor = '#f39c12';

        // Connect to device
        bleServer = await bleDevice.gatt.connect();
        bleService = await bleServer.getPrimaryService(SERVICE_UUID);
        bleCharacteristic = await bleService.getCharacteristic(CHARACTERISTIC_UUID);

        // Enable notifications
        await bleCharacteristic.startNotifications();
        bleCharacteristic.addEventListener('characteristicvaluechanged', handleCharacteristicValueChanged);

        // Update UI
        isConnected = true;
        bleStatus.textContent = 'Connected';
        bleStatus.style.backgroundColor = '#27ae60';
        deviceName.textContent = bleDevice.name || 'ESP32 Device';
        
        connectBtn.disabled = true;
        disconnectBtn.disabled = false;

        console.log('BLE Connected successfully!');

    } catch (error) {
        console.error('BLE Connection failed:', error);
        bleStatus.textContent = 'Connection Failed';
        bleStatus.style.backgroundColor = '#e74c3c';
        alert('Connection failed: ' + error.message);
    }
}

// Handle incoming BLE data
function handleCharacteristicValueChanged(event) {
    const value = event.target.value;
    const decoder = new TextDecoder('utf-8');
    const data = decoder.decode(value);
    
    console.log('Received BLE data:', data);
    parseBLEData(data);
}

// Parse BLE data from ESP32
function parseBLEData(data) {
    console.log('Raw BLE data:', data);
    
    // Split by lines and process each line
    const lines = data.split('\n');
    
    lines.forEach(line => {
        line = line.trim();
        if (line && line.includes(':')) {
            const colonIndex = line.indexOf(':');
            const key = line.substring(0, colonIndex).trim();
            const value = line.substring(colonIndex + 1).trim();
            
            console.log(`Parsing: key="${key}", value="${value}"`);
            
            switch (key) {
                case 'elevation':
                    const elevation = parseFloat(value);
                    if (!isNaN(elevation)) {
                        sensorData.elevation = elevation;
                        document.getElementById('elevation').textContent = elevation.toFixed(2);
                        console.log('Updated elevation:', elevation);
                    }
                    break;
                    
                case 'distance':
                    const distance = parseFloat(value);
                    if (!isNaN(distance)) {
                        sensorData.distance = distance;
                        document.getElementById('distance').textContent = distance.toFixed(1);
                        console.log('Updated distance:', distance);
                    }
                    break;
                    
                case 'roll':
                    const roll = parseFloat(value);
                    if (!isNaN(roll)) {
                        sensorData.roll = roll;
                        document.getElementById('roll').textContent = roll.toFixed(2);
                        console.log('Updated roll:', roll);
                    }
                    break;
                    
                case 'yaw':
                    const yaw = parseFloat(value);
                    if (!isNaN(yaw)) {
                        sensorData.yaw = yaw;
                        document.getElementById('yaw').textContent = yaw.toFixed(2);
                        console.log('Updated yaw:', yaw);
                    }
                    break;
                    
                case 'mode':
                    const mode = parseInt(value);
                    if (!isNaN(mode)) {
                        sensorData.mode = mode;
                        document.getElementById('mode').textContent = mode;
                        console.log('Updated mode:', mode);
                    }
                    break;
                    
                default:
                    console.log('Unknown key:', key, 'value:', value);
            }
        }
    });
    
    // Force 3D update
    if (cube && renderer && scene && camera) {
        const elevationRad = degToRad(sensorData.elevation - zeroOffsets.elevation);
        const rollRad = degToRad(sensorData.roll - zeroOffsets.roll);
        const yawRad = degToRad(sensorData.yaw - zeroOffsets.yaw);
        
        cube.rotation.x = elevationRad;
        cube.rotation.y = yawRad;
        cube.rotation.z = rollRad;
        
    renderer.render(scene, camera);
        console.log('3D rotation updated:', { elevationRad, rollRad, yawRad });
    }
}

// Disconnect BLE
async function disconnectBLE() {
    try {
        if (bleCharacteristic) {
            await bleCharacteristic.stopNotifications();
        }
        if (bleServer) {
            await bleServer.disconnect();
        }
        
        isConnected = false;
        bleStatus.textContent = 'Disconnected';
        bleStatus.style.backgroundColor = '#e74c3c';
        
        connectBtn.disabled = false;
        disconnectBtn.disabled = true;
        
        console.log('BLE Disconnected');
        
    } catch (error) {
        console.error('Disconnect error:', error);
    }
}

// ลบฟังก์ชัน resetPosition ออกแล้ว

// Event listeners
connectBtn.addEventListener('click', connectBLE);
disconnectBtn.addEventListener('click', disconnectBLE);
window.addEventListener('resize', onWindowResize, false);

// Initialize when page loads
document.addEventListener('DOMContentLoaded', function() {
    console.log('Page loaded, initializing...');
    
    // Wait a bit for Three.js to load
    setTimeout(() => {
        init3D();
    }, 1000);
    
    // Check if BLE is available
    if (!navigator.bluetooth) {
        connectBtn.disabled = true;
        connectBtn.textContent = 'BLE Not Supported';
        bleStatus.textContent = 'Not Available';
        bleStatus.style.backgroundColor = '#95a5a6';
    }
    
    // ลบปุ่ม Test 3D ออกแล้ว
});

// Handle page unload
window.addEventListener('beforeunload', function() {
    if (isConnected) {
        disconnectBLE();
    }
});
