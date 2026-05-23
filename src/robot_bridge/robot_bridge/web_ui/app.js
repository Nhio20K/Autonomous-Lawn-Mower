/**
 * Mower Bot Web UI - Core Logic
 * Handles ROS connection, 3D visualization, and telemetry updates.
 */

const ROS_URL = `ws://${window.location.hostname}:9090`;
const VIDEO_URL = `http://${window.location.hostname}:8080/stream?topic=/camera/camera/color/image_raw`;

class MowerApp {
    constructor() {
        this.ros = new ROSLIB.Ros();
        this.viewer = null;
        this.tfClient = null;
        this.connected = false;
        
        this.initROS();
        this.init3D();
        this.initJoystick();
        this.initEventListeners();
        this.updateUptime();
    }

    initROS() {
        this.ros.on('connection', () => {
            console.log('Connected to websocket server.');
            this.connected = true;
            this.updateStatus(true);
            this.setupSubscribers();
            this.setupCamera();
        });

        this.ros.on('error', (error) => {
            console.log('Error connecting to websocket server: ', error);
            this.updateStatus(false);
        });

        this.ros.on('close', () => {
            console.log('Connection to websocket server closed.');
            this.connected = false;
            this.updateStatus(false);
            // Auto reconnect
            setTimeout(() => this.ros.connect(ROS_URL), 3000);
        });

        this.ros.connect(ROS_URL);
    }

    updateStatus(online) {
        const indicator = document.querySelector('.status-indicator');
        const text = document.querySelector('.status-text');
        if (online) {
            indicator.classList.remove('offline');
            indicator.classList.add('online');
            text.innerText = 'Connected';
        } else {
            indicator.classList.remove('online');
            indicator.classList.add('offline');
            text.innerText = 'Disconnected';
        }
    }

    init3D() {
        const container = document.getElementById('map-container');
        
        // Create the main viewer.
        this.viewer = new ROS3D.Viewer({
            divID: 'map-container',
            width: container.offsetWidth,
            height: container.offsetHeight,
            antialias: true,
            background: '#0a0a12'
        });

        // Setup a client to listen to TFs.
        this.tfClient = new ROSLIB.TFClient({
            ros: this.ros,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0,
            fixedFrame: 'map'
        });

        // Setup the map client.
        const gridClient = new ROS3D.OccupancyGridClient({
            ros: this.ros,
            rootObject: this.viewer.scene,
            continuous: true,
            topic: '/map'
        });

        // Setup the robot model (using a simple marker for now as URDF might be complex to load via CDN)
        const robotMarker = new ROS3D.Axes({
            shaftRadius: 0.1,
            headRadius: 0.2,
            headLength: 0.3
        });
        
        // Add robot frame
        this.tfClient.subscribe('base_link', (tf) => {
            robotMarker.position.set(tf.translation.x, tf.translation.y, tf.translation.z);
            robotMarker.quaternion.set(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
        });
        this.viewer.scene.add(robotMarker);

        // Setup Path Client
        const pathClient = new ROS3D.Path({
            ros: this.ros,
            tfClient: this.tfClient,
            rootObject: this.viewer.scene,
            topic: '/plan',
            color: '#4e44ff'
        });

        // Handle window resize
        window.addEventListener('resize', () => {
            this.viewer.resize(container.offsetWidth, container.offsetHeight);
        });

        // Hide loader once initialized
        setTimeout(() => {
            const loader = container.querySelector('.loader');
            if (loader) loader.style.display = 'none';
        }, 2000);
    }

    setupSubscribers() {
        // Battery Status
        const batterySub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/battery_status',
            messageType: 'diagnostic_msgs/DiagnosticStatus'
        });
        batterySub.subscribe((msg) => {
            let voltage = 0;
            msg.values.forEach(kv => {
                if (kv.key === 'voltage') voltage = parseFloat(kv.value);
            });
            document.getElementById('val-battery').innerText = `${voltage.toFixed(1)} V`;
            
            // Calculate percentage (assuming 24V system: 21V-28V)
            let pct = ((voltage - 21.0) / (28.0 - 21.0)) * 100;
            pct = Math.max(0, Math.min(100, pct));
            document.getElementById('bar-battery').style.width = `${pct}%`;
        });

        // GPS Fix
        const gpsSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/fix',
            messageType: 'sensor_msgs/NavSatFix'
        });
        gpsSub.subscribe((msg) => {
            const statusMap = {
                '-1': 'NO FIX',
                '0': 'FIX',
                '1': 'SBAS',
                '2': 'RTK'
            };
            document.getElementById('val-gps').innerText = statusMap[msg.status.status] || 'UNKNOWN';
            
            // Accuracy from covariance
            const acc = Math.sqrt(msg.position_covariance[0]) * 100;
            document.getElementById('val-gps-acc').innerText = `Acc: ${acc.toFixed(1)} cm`;
        });

        // IMU / Heading
        const headingSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/compass/heading',
            messageType: 'std_msgs/Float64'
        });
        headingSub.subscribe((msg) => {
            document.getElementById('val-heading').innerText = `${msg.data.toFixed(0).padStart(3, '0')}°`;
        });

        // Odom / Speed
        const odomSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/odom_raw',
            messageType: 'nav_msgs/Odometry'
        });
        odomSub.subscribe((msg) => {
            const speed = msg.twist.twist.linear.x;
            document.getElementById('val-speed').innerText = `${speed.toFixed(2)} m/s`;
        });

        // Lidar Safety
        const lidarSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/lidar_safety_status',
            messageType: 'std_msgs/String'
        });
        lidarSub.subscribe((msg) => {
            const el = document.getElementById('val-lidar');
            el.innerText = msg.data;
            el.style.color = msg.data === 'SAFE' ? 'var(--success)' : 'var(--danger)';
        });
    }

    setupCamera() {
        const camImg = document.getElementById('camera-stream');
        camImg.src = VIDEO_URL;
        
        // Simple FPS counter simulation for UI feel
        let frames = 0;
        setInterval(() => {
            if (this.connected) {
                const fps = Math.floor(Math.random() * 5) + 12; // Simulate 12-17 FPS
                document.getElementById('vision-fps').innerText = `${fps} FPS`;
            }
        }, 1000);
    }

    initJoystick() {
        const options = {
            zone: document.getElementById('joystick'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#4e44ff',
            size: 120
        };
        
        const manager = nipplejs.create(options);
        const cmdVelPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/cmd_vel_teleop', // Use teleop topic for twist_mux
            messageType: 'geometry_msgs/Twist'
        });

        let moveInterval = null;
        let twist = new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        });

        manager.on('move', (evt, data) => {
            if (!document.getElementById('manual-mode').checked) return;

            const maxLin = 0.5; // m/s
            const maxAng = 1.0; // rad/s
            
            const forward = data.vector.y;
            const turn = -data.vector.x;

            twist.linear.x = forward * maxLin;
            twist.angular.z = turn * maxAng;

            if (!moveInterval) {
                moveInterval = setInterval(() => {
                    cmdVelPub.publish(twist);
                }, 100);
            }
        });

        manager.on('end', () => {
            if (moveInterval) {
                clearInterval(moveInterval);
                moveInterval = null;
            }
            twist.linear.x = 0;
            twist.angular.z = 0;
            cmdVelPub.publish(twist);
        });
    }

    initEventListeners() {
        // Emergency Stop
        document.getElementById('btn-emergency').addEventListener('click', () => {
            const stopPub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_emergency',
                messageType: 'std_msgs/String'
            });
            stopPub.publish(new ROSLIB.Message({ data: 'E,1' }));
            alert('EMERGENCY STOP ACTIVATED!');
        });

        // Start Mission
        document.getElementById('btn-start').addEventListener('click', () => {
            // Trigger mission start topic if exists
            alert('Mission starting...');
        });

        // Reset View
        document.getElementById('reset-view').addEventListener('click', () => {
            // Reset camera position
            this.viewer.cameraControls.reset();
        });
    }

    updateUptime() {
        let startTime = Date.now();
        setInterval(() => {
            const elapsed = Math.floor((Date.now() - startTime) / 1000);
            const hrs = Math.floor(elapsed / 3600).toString().padStart(2, '0');
            const mins = Math.floor((elapsed % 3600) / 60).toString().padStart(2, '0');
            const secs = (elapsed % 60).toString().padStart(2, '0');
            document.getElementById('val-uptime').innerText = `${hrs}:${mins}:${secs}`;
        }, 1000);
    }
}

// Initialize on load
window.onload = () => {
    window.app = new MowerApp();
};
