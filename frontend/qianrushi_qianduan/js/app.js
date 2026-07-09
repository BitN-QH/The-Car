/**
 * RDK X5 闂佺绻掗弻澶愭閳哄懎鐤惧Δ锕€鐏濋崢鎾煕韫囧濡奸柟顔界矒楠炴牠顢楅崘顏嗩吋闂佸搫鐗嗛幖顐⑩枍閹烘挾顩?- Web 闁哄鏅滅划搴ㄥ煝婵傜绠崇憸宥夊春濡や胶鈻旈幖娣€楃粻銉х磽娴ｈ缍戦梺瑙ｆ櫊瀹曪綁鎮ч崼銏狀棎闂婎偄娲ら崯鎾焵椤掍焦顫楃紒?
 */

// ==========================================================================
// 1. 闂佺绻堥崝宀勬儑椤掑嫭鍋愰柤鍝ヮ暯閸嬫挻鎷呮搴ｆ喛闂備焦婢樼粔鍫曟偪?
// ==========================================================================
const CONFIG = {
    loopRateMs: 50,          // 婵炴垶鎹侀褔骞嗘繝鍥ㄥ仢妞ゆ牜鍎愰弳銉╂煟?(20Hz)
    odomRateMs: 100,         // 闂備焦褰冮惉濂稿煝閼测晜濯奸柍鈺佸暞缁绢垶鏌￠崒娆忓祮妞わ絽鐖奸幃?(10Hz)
    telemetryRateMs: 1000,   // 闂備緡鍏欓崕鑼矈閹绢喖鏋侀柣妤€鐗嗙粊锕傛煛閸パ呮憼闁哄苯锕ラ敍鎰板箣閻樿尪濮?(1Hz)
    mapScaleDefault: 4.0,    // 婵帗绋掗…鍫ヮ敇婵犳艾鎹堕柡澶嬪缁傚牏绱撻崒妤佹珕闁哄倷绀佽闁哄倸澧芥导?
    vlmTypingSpeedMs: 30,    // VLM闂佽鍓氱换鍕庨鈧弻褔鎮欓鈧埅鐢告倵濞戞瑯娈曟繝褉鍋撻梻渚囧亞閸犲海鈧?
    rdkIp: "192.168.39.26",
    rdkHttpBase: "http://192.168.39.26:8080",
    simStepScales: [0.1, 0.2, 0.35, 0.5, 1.0],
    simStepScaleDefaultIndex: 1,
    vlmRosBridge: "ws://192.168.39.26:9090",
};

const state = {
    // Connection state
    isSimMode: true,
    isConnected: false,
    isVlmConnected: false,
    ros: null,
    rdkHttpBase: CONFIG.rdkHttpBase,
    yoloEvents: null,
    yoloPollTimer: null,
    lastRobotCommand: "",
    lastMissionState: "",
    
    // 闂佸搫鐗嗛幖顐⑩枍閹烘挾顩查弶鐐村缁夋潙鈹戦绗轰粧缂佹鏈濠氬箛椤掆偓琚?(X闁哄鍋犻幓顏嗘娴兼潙绀堢€广儱鎳忛崐鐢告煥濞戞﹩鍔ラ柡澶屽仩閹活亞妲愰弶娆惧晠闁挎洍鍋撶憸鎵焾铻ｆい蹇撴煀閳哄懏鏅悗娑樺悑濞?Yaw闂佹寧绋掔喊宥呂涢崱妯诲?
    pose: { x: 0.0, y: 0.0, yaw: 0.0 }, // 閻庢鍟崘顭戝敽闂佸憡顨嗗ú鎴犵礊?
    targetPose: { x: 0.0, y: 0.0, yaw: 0.0 }, // 閻庢鍟崘顭戝敽闂佸憡顨嗗ú鎴犵礊?
    speed: { vx: 0.0, vy: 0.0, wz: 0.0 },
    simStepScaleIndex: CONFIG.simStepScaleDefaultIndex,
    simStepScale: CONFIG.simStepScales[CONFIG.simStepScaleDefaultIndex],
    speedLimit: 0.5,         // 缂備焦鍎冲锟犲焵椤掑倸鏋庨悗纭呮珪缁嬪绻濋崶顭戔偓?(m/s)
    yawLimit: 0.8,           // 闁荤喐鐟︾敮锟犲焵椤掑倸鏋庨悗纭呮珪缁嬪绻濋崶顭戔偓?(rad/s)
    
    // Forklift state
    forklift: {
        height: 0,           // 0 - 100%
        targetHeight: 0,
        payload: 0,          // 闁哄鍋涢埀顒傚枎缁?(kg)
        statusText: "缂備礁鐭傜紓姘?(0 kg)"
    },
    
    // 婵炵鍋愭慨鐢稿礉閸涙潙闂柕濞垮€楅悷銏ゆ⒑椤戞儳鍔滅紒?
    battery: { voltage: 16.2, percent: 85 },
    rdkTelemetry: { cpu: 28, ram: 42, temp: 45 },
    stm32Telemetry: { motors: [0, 0, 0, 0], uartOk: true },
    
    // 闂侀潻闄勫妯好瑰Ο鑽ゎ洸闁靛牆瀚棄宥夋煙鐠ㄥ鍟悡?(濡ょ姷鍋涚壕顓濈昂婵炴垶鎸稿ù椋庣磽婢舵劕缁?
    map: {
        scale: CONFIG.mapScaleDefault,
        offsetX: 0,
        offsetY: 0,
        isDragging: false,
        dragStartX: 0,
        dragStartY: 0,
        gridSize: 500, // 闂佸搫绉撮幊蹇涙偋閹间礁鎹堕柡澶嬪缁傚牓鏌涘鍛缂佲偓瀹€鈧禍鎼佸传閸曢潧娈?(25m x 25m, 5cm闂佸憡甯掑Λ婊勩仈閹间焦鍋?
        resolution: 0.05,
        originX: -12.5, // 闂佺粯銇涢弲婵嬪箖婵犲洤鍌ㄩ柣鏂挎惈娴?
        originY: -12.5
    },
    
    // YOLO 闁荤喐鐟ュΛ婊堬綖?
    vision: {
        cameraPan: 0.0, // 闂佺儵鏅涢幉鈥斥攦閳ь剛鈧綊娼荤粻鎴ｃ亹閹间礁绫嶉悗锝庡幗缁侇喗淇婇妤€澧查悗?
        detectedObjects: [],
        rawDetections: [],
        imageWidth: 640,
        imageHeight: 480,
        yoloSource: "primary",
    },
    
    // VLM state
    vlm: {
        isThinking: false,
        reasoningSteps: "",
        chatHistory: [],
        lastResponse: "",
        activeBotMessageEl: null,
        chatMode: "model"
    },
    
    // 婵炲濮鹃褎鎱ㄩ悢鍏尖挀闁绘柨鍢查悘?
    tasks: [],
    activeTaskIndex: -1,
    isTaskRunning: false,
    
    // Keyboard input state
    keys: { w: false, a: false, s: false, d: false, q: false, e: false }
};

// ==========================================================================
// 2. DOM 闂佺绻愰崯鎵矆瀹€鍕殧鐎瑰嫭婢樼徊?
// ==========================================================================
const dom = {
    // Connection controls
    connectionBadge: document.getElementById("connection-badge"),
    connectionStatusText: document.getElementById("connection-status-text"),
    estopBtn: document.getElementById("estop-btn"),
    
    // 闁哄鏅濋崑鐐垫暜閹绢喗鐓€鐎广儱娲ㄩ弸?
    rdkIpInput: document.getElementById("rdk-ip"),
    applyRdkIpBtn: document.getElementById("apply-rdk-ip-btn"),
    rdkHttpInput: document.getElementById("rdk-http-url"),
    rosIpInput: document.getElementById("ros-ip"),
    videoUrlInput: document.getElementById("video-url"),
    connectBtn: document.getElementById("connect-btn"),
    
    // 闂佸綊娼ч鍛叏閳哄懏鐒奸柕澶涚畱娴?
    valVx: document.getElementById("val-vx"),
    valVy: document.getElementById("val-vy"),
    valWz: document.getElementById("val-wz"),
    speedLimitRange: document.getElementById("speed-limit"),
    speedLimitVal: document.getElementById("val-speed-limit"),
    yawLimitRange: document.getElementById("yaw-limit"),
    yawLimitVal: document.getElementById("val-yaw-limit"),
    joystickZone: document.getElementById("joystick-zone"),
    
    // 闂佸憡鐟ラˇ鍐测攦閸涙潙瀚夐柣鎴烆焽閳?
    forkliftHeightRange: document.getElementById("forklift-height"),
    forkliftHeightVal: document.getElementById("forklift-height-val"),
    forkliftPayloadVal: document.getElementById("forklift-payload-val"),
    forkLiftBtn: document.getElementById("fork-lift-btn"),
    forkLowerBtn: document.getElementById("fork-lower-btn"),
    
    mapCanvas: document.getElementById("map-canvas"),
    mapZoomIn: document.getElementById("map-zoom-in"),
    mapZoomOut: document.getElementById("map-zoom-out"),
    mapReset: document.getElementById("map-reset"),
    simStepScaleBtn: document.getElementById("sim-step-scale"),
    robotX: document.getElementById("robot-x"),
    robotY: document.getElementById("robot-y"),
    robotYaw: document.getElementById("robot-yaw"),
    targetX: document.getElementById("target-x"),
    targetY: document.getElementById("target-y"),
    targetYaw: document.getElementById("target-yaw"),
    navStartBtn: document.getElementById("nav-start-btn"),
    navCancelBtn: document.getElementById("nav-cancel-btn"),
    slamSaveBtn: document.getElementById("slam-save-btn"),
    
    // VLM 闂佸搫鎳樼紓姘跺礂濮椻偓瀹曟繈鍩￠崘鈺佲偓?
    chatMessages: document.getElementById("chat-messages"),
    chatInput: document.getElementById("chat-input"),
    chatSendBtn: document.getElementById("chat-send-btn"),
    chatModeModel: document.getElementById("chat-mode-model"),
    chatModeVlm: document.getElementById("chat-mode-vlm"),
    vlmReasoningContent: document.getElementById("vlm-reasoning-content"),
    
    // YOLO 闁荤喐鐟ュΛ婊堬綖?
    visionCanvas: document.getElementById("vision-canvas"),
    visionCanvasAux: document.getElementById("vision-canvas-aux"),
    videoStreamImg: document.getElementById("video-stream-img"),
    videoStreamImgAux: document.getElementById("video-stream-img-aux"),
    videoDisconnectOverlay: document.getElementById("video-disconnect-overlay"),
    videoDisconnectOverlayAux: document.getElementById("video-disconnect-overlay-aux"),
    yoloSourcePrimary: document.getElementById("yolo-source-primary"),
    yoloSourceAux: document.getElementById("yolo-source-aux"),
    visionObjectList: document.getElementById("vision-object-list"),
    
    // 闂備緡鍏欓崕鑼矈閹绢喖鏋侀柣妤€鐗嗙粊?
    rdkCpu: document.getElementById("rdk-cpu"),
    rdkCpuBar: document.getElementById("rdk-cpu-bar"),
    rdkRam: document.getElementById("rdk-ram"),
    rdkRamBar: document.getElementById("rdk-ram-bar"),
    rdkTemp: document.getElementById("rdk-temp"),
    rdkTempBar: document.getElementById("rdk-temp-bar"),
    stmYaw: document.getElementById("stm-yaw"),
    stmOdom: document.getElementById("stm-odom"),
    stmMotors: document.getElementById("stm-motors"),
    stmUartStatus: document.getElementById("stm-uart-status"),
    batteryPercent: document.getElementById("battery-percent"),
    batteryVoltage: document.getElementById("battery-voltage"),
    batteryBar: document.getElementById("battery-bar"),
    batterySvg: document.getElementById("battery-svg"),
    
    // 闁圭厧鐡ㄥú鐔煎磿鐎涙顩烽悹鍥ㄥ絻椤倝骞栭弶鎴犵闁?
    btnAddNav: document.getElementById("btn-add-nav"),
    btnAddSearch: document.getElementById("btn-add-search"),
    btnAddLift: document.getElementById("btn-add-lift"),
    btnAddDrop: document.getElementById("btn-add-drop"),
    btnAddPatrol: document.getElementById("btn-add-patrol"),
    taskRunBtn: document.getElementById("task-run-btn"),
    taskPauseBtn: document.getElementById("task-pause-btn"),
    taskClearBtn: document.getElementById("task-clear-btn"),
    tasksTimeline: document.getElementById("tasks-timeline"),
    
    // Alerts and logs
    alertBanner: document.getElementById("alert-banner"),
    alertMessage: document.getElementById("alert-message"),
    logContainer: document.getElementById("log-container"),
    logClearBtn: document.getElementById("log-clear-btn")
};

// ==========================================================================
// 3. 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宥忓閻熴垹霉濠婂喚鍎庢繛鍡愬灲閹嫰骞嬪┑鍥у壎缂傚倷鐒﹂崹鐢告偩?
// ==========================================================================
window.addEventListener("DOMContentLoaded", () => {
    initUI();
    initJoystick();
    initKeyboard();
    initMapCanvas();
    
    // 闂佸憡鍑归崹鐗堟叏閳哄啫瀵查柤濮愬€楅崺鐘碘偓鐢稿亰娴滅偤鐛?
    setInterval(updateLoop, CONFIG.loopRateMs);
    setInterval(updateOdomLoop, CONFIG.odomRateMs);
    setInterval(updateTelemetryLoop, CONFIG.telemetryRateMs);
    
    addLog("INFO", "SYSTEM", "System ready. Simulation and RDK control are enabled.");
});

// UI initialization
function initUI() {
    applyRdkIpToConnectionFields(false);

    state.isSimMode = true;
    // 闁哄鏅濋崑鐐垫暜閹绢喖绠板鑸靛姈鐏?
    dom.connectBtn.addEventListener("click", () => {
        if (state.isConnected) {
            disconnectFromRos();
        } else {
            connectToRos();
        }
    });

    dom.applyRdkIpBtn.addEventListener("click", () => {
        applyRdkIpToConnectionFields(true);
    });

    dom.rdkIpInput.addEventListener("keydown", (e) => {
        if (e.key === "Enter") {
            e.preventDefault();
            applyRdkIpToConnectionFields(true);
        }
    });

    document.querySelectorAll(".drive-btn").forEach((button) => {
        button.addEventListener("click", () => {
            sendDriveCommand(button.dataset.command);
        });
    });

    document.querySelectorAll(".source-btn").forEach((button) => {
        button.addEventListener("click", () => {
            setYoloSource(button.dataset.source);
        });
    });

    // 闂備緡鍋嗛崰搴ｂ偓瑙勫▕濮婁粙骞囬鈧悡鎴炵節婵犲啫鐏ユ繛?
    dom.speedLimitRange.addEventListener("input", (e) => {
        state.speedLimit = parseFloat(e.target.value);
        dom.speedLimitVal.innerText = state.speedLimit.toFixed(1);
    });
    dom.yawLimitRange.addEventListener("input", (e) => {
        state.yawLimit = parseFloat(e.target.value);
        dom.yawLimitVal.innerText = state.yawLimit.toFixed(1);
    });

    // 闂佸憡鐟ラˇ鍐测攦閸涙潙瀚夐柣鎴烆焽閳ь剦鍨卞蹇涘箻閸愭彃姹查梺纭呯焽閸愩劎鍘?
    dom.forkliftHeightRange.addEventListener("input", (e) => {
        const val = parseInt(e.target.value);
        state.forklift.targetHeight = val;
        sendForkliftCommand(val);
    });
    
    dom.forkLiftBtn.addEventListener("click", () => {
        dom.forkliftHeightRange.value = 100;
        state.forklift.targetHeight = 100;
        sendForkliftCommand(100);
        addLog("INFO", "ACTUATOR", "Action updated.");
    });
    dom.forkLowerBtn.addEventListener("click", () => {
        dom.forkliftHeightRange.value = 0;
        state.forklift.targetHeight = 0;
        sendForkliftCommand(0);
        addLog("INFO", "ACTUATOR", "Action updated.");
    });

    // 闂侀潻闄勫妯好瑰Ο鍏煎枂闁糕剝鍑瑰鈥愁熆鐠鸿櫣小缂?
    dom.mapReset.addEventListener("click", resetMapView);
    dom.mapZoomIn.addEventListener("click", () => { state.map.scale *= 1.2; drawMap(); });
    dom.mapZoomOut.addEventListener("click", () => { state.map.scale /= 1.2; drawMap(); });
    if (dom.simStepScaleBtn) {
        dom.simStepScaleBtn.addEventListener("click", cycleSimStepScale);
        updateSimStepScaleButton();
    }

    // 闁诲簼绲绘竟鍫ュ春閸涙潙绠崇憸宥夊春濡ゅ懎绠板鑸靛姈鐏?
    dom.navStartBtn.addEventListener("click", () => {
        const tx = parseFloat(dom.targetX.value);
        const ty = parseFloat(dom.targetY.value);
        const tyaw = parseFloat(dom.targetYaw.value) * Math.PI / 180;
        
        state.targetPose.x = tx;
        state.targetPose.y = ty;
        state.targetPose.yaw = tyaw;
        
        sendNavigationGoal(tx, ty, tyaw);
    });
    
    dom.navCancelBtn.addEventListener("click", () => {
        cancelNavigation();
    });

    dom.slamSaveBtn.addEventListener("click", () => {
        saveSlamMap();
    });

    // VLM 闂佸搫鎳樼紓姘跺礂濮椻偓瀹曟繈鍩￠崘鈺佲偓?
    dom.chatSendBtn.addEventListener("click", handleVlmChatInput);
    dom.chatInput.addEventListener("keydown", (e) => {
        if (e.key === "Enter" && !e.shiftKey) {
            e.preventDefault();
            handleVlmChatInput();
        }
    });
    document.querySelectorAll(".chat-mode-btn").forEach((button) => {
        button.addEventListener("click", () => setChatMode(button.dataset.mode));
    });
    setChatMode(state.vlm.chatMode);

    // 闁圭厧鐡ㄥú鐔煎磿鐎涙顩烽悹鍥ㄥ絻椤倝骞栭弶鎴犵闁割煈浜幃浠嬫偄缁嬭法浜ｉ梺鍦焾椤︾敻骞?
    dom.btnAddNav.addEventListener("click", () => addTask("NAV", { x: 2.0, y: -1.0, yaw: 90 }));
    dom.btnAddSearch.addEventListener("click", () => addTask("SEARCH", { target: "pallet" }));
    dom.btnAddLift.addEventListener("click", () => addTask("LIFT", {}));
    dom.btnAddDrop.addEventListener("click", () => addTask("DROP", {}));
    dom.btnAddPatrol.addEventListener("click", () => addTask("PATROL", { waypoints: [[1.5,0.5], [1.5,-1.5], [-1.0,-1.5], [-1.0,0.5]] }));

    dom.taskRunBtn.addEventListener("click", startTaskQueue);
    dom.taskPauseBtn.addEventListener("click", pauseTaskQueue);
    dom.taskClearBtn.addEventListener("click", clearTaskQueue);
    
    // 闂佸搫鍟ㄩ崕杈╂崲閺傝￥鈧帡宕ㄩ娑樷偓?
    dom.logClearBtn.addEventListener("click", () => {
        dom.logContainer.innerHTML = "";
    });

    // E-STOP 闂佽鍏欓崕杈ㄧ?
    dom.estopBtn.addEventListener("click", triggerEStop);
}

function normalizeRdkIpInput(value) {
    return (value || CONFIG.rdkIp)
        .trim()
        .replace(/^https?:\/\//, "")
        .replace(/\/.*$/, "")
        .replace(/:\d+$/, "");
}

function applyRdkIpToConnectionFields(shouldLog = true) {
    const ip = normalizeRdkIpInput(dom.rdkIpInput.value);
    dom.rdkIpInput.value = ip;
    dom.rdkHttpInput.value = `http://${ip}:8080`;
    dom.rosIpInput.value = `http://${ip}:8080/api/vlm`;
    dom.videoUrlInput.value = `http://${ip}:8080/stream.mjpg`;
    if (shouldLog) {
        addLog("INFO", "COMM", `閻庣懓鎲¤ぐ鍐€冨鍫熷仺闁靛鍎查崟楣冩煟閳╁啰鐭嬫繛?IP: ${ip}`);
    }
}

// ==========================================================================
// 4. ROS2 WiFi 闂備緡鍋呮穱娲敊?(roslibjs)
// ==========================================================================
let rosTopics = {};

function normalizeBaseUrl(url) {
    return url.trim().replace(/\/+$/, "");
}

function apiUrl(path) {
    return `${state.rdkHttpBase}${path}`;
}

async function connectToRos() {
    const httpUrl = normalizeBaseUrl(dom.rdkHttpInput.value || CONFIG.rdkHttpBase);
    if (!httpUrl) {
        addLog("ERROR", "COMM", "Action updated.");
        return;
    }

    state.rdkHttpBase = httpUrl;
    addLog("INFO", "COMM", `濠殿喗绻愮徊钘夛耿椤忓懏浜ら柣鎰綑婢跺秹鏌涢敂钘夘棆闁硅姤鍔曠叅?HTTP 闂佸搫鐗嗙粔瀛樻叏? ${httpUrl}...`);
    dom.connectBtn.innerText = "闁哄鏅濋崑鐐垫暜鐎涙鈻?..";
    dom.connectBtn.disabled = true;

    try {
        await fetchRobotState();
        state.isConnected = true;
        dom.connectBtn.innerText = "Text";
        dom.connectBtn.className = "btn btn-danger";
        dom.connectBtn.disabled = false;
        dom.connectionBadge.className = "badge badge-connected";
        dom.connectionStatusText.innerText = "Text";

        const videoUrl = dom.videoUrlInput.value.trim() || apiUrl("/stream.mjpg");
        const auxVideoUrl = apiUrl("/stream_aux.mjpg");
        dom.videoStreamImg.src = videoUrl;
        dom.videoStreamImgAux.src = auxVideoUrl;
        dom.videoStreamImg.classList.remove("hidden");
        dom.videoStreamImgAux.classList.remove("hidden");
        dom.videoDisconnectOverlay.style.display = "none";
        dom.videoDisconnectOverlayAux.style.display = "none";

        startYoloStateStream();
        state.isVlmConnected = true;
        addLog("SUCCESS", "VLM", "Action updated.");
        addLog("SUCCESS", "COMM", "Action updated.");
    } catch (e) {
        state.isConnected = false;
        addLog("ERROR", "COMM", `闂侀潻闄勬竟鍡涘箺閹邦優?HTTP 闁哄鏅濋崑鐐垫暜鐎涙ê绶為弶鍫亯琚? ${e.message}`);
        resetRosUI();
    }
}

function disconnectFromRos() {
    stopYoloStateStream();
    if (state.ros) {
        state.ros.close();
        state.ros = null;
    }
    state.isVlmConnected = false;
    rosTopics = {};
    resetRosUI();
}

function resetRosUI() {
    dom.connectBtn.innerText = "Text";
    dom.connectBtn.className = "btn btn-primary";
    dom.connectBtn.disabled = false;
    state.isConnected = false;
    dom.connectionBadge.className = "badge badge-disconnected";
    dom.connectionStatusText.innerText = "Text";
    dom.videoStreamImg.classList.add("hidden");
    dom.videoStreamImg.src = "";
    dom.videoStreamImgAux.classList.add("hidden");
    dom.videoStreamImgAux.src = "";
    dom.videoDisconnectOverlay.style.display = "flex";
    dom.videoDisconnectOverlayAux.style.display = "flex";
}

// 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宓啰宀涢柣銏╁灠閸熸澘鈻撻幋鐘冲珰婵犻潧顑愰弳?
function initRosPublishers() {
    rosTopics.goalPose = new ROSLIB.Topic({
        ros: state.ros,
        name: '/goal_pose',
        messageType: 'geometry_msgs/PoseStamped'
    });
    
    rosTopics.forkliftCmd = new ROSLIB.Topic({
        ros: state.ros,
        name: '/forklift_cmd',
        messageType: 'std_msgs/Int32'
    });
    
    rosTopics.vlmCmd = new ROSLIB.Topic({
        ros: state.ros,
        name: '/prompt_text',
        messageType: 'std_msgs/String'
    });
}

// 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宥庡墻閸氬倿姊婚崘銊﹀櫧婵炲牊鍨归幏鐘测攽鐎ｅ灚姣?
function initRosSubscribers() {
    // Odometry subscriber
    const odomSub = new ROSLIB.Topic({
        ros: state.ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });
    odomSub.subscribe((message) => {
        if (state.isSimMode) return; // 婵炲濮惧▔鏇烇耿閳ユ緞鐔煎灳瀹曞洨顢呮繛鎴炴尭椤戝嫭绻涢崶顒佸仺闁靛鍔嶉悵鎾绘煙妞嬪骸鏋涢柛锝夘棑缁瑧鈧綆鍙庨崥鈧?
        // 闁荤喐鐟辩徊楣冩倵娴犲閿ら柟閭﹀幘閸?
        state.pose.x = message.pose.pose.position.x;
        state.pose.y = message.pose.pose.position.y;
        
        // 闁荤喐鐟辩徊楣冩倵娴犲鐐婃繛鎴灻敮妤呮煛娴ｈ绶查崡蹇涙煙椤戞寧顦烽悹?Yaw
        const q = message.pose.pose.orientation;
        state.pose.yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        // 闁荤姴娲╅褑銇愰崶顒€鐭楃€广儱顦藉ú顖炴⒑椤愩倕鏋庨悗?
        state.speed.vx = message.twist.twist.linear.x;
        state.speed.vy = message.twist.twist.linear.y;
        state.speed.wz = message.twist.twist.angular.z;
    });
    
    // 闁荤姳闄嶉崹钘壩ｉ崟顖氭嵍闁哄瀵х粋?
    const mapSub = new ROSLIB.Topic({
        ros: state.ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
    });
    mapSub.subscribe((message) => {
        if (state.isSimMode) return;
        addLog("INFO", "MAP", "成功接收来自真实机器人的地图数据。");
        // TODO: parse OccupancyGrid and render the real map when needed.
    });
    
    // 闁荤姳闄嶉崹钘壩?VLM 闂佹悶鍎抽崑娑⑺?
    const vlmResponseSub = new ROSLIB.Topic({
        ros: state.ros,
        name: '/tts_text',
        messageType: 'std_msgs/String'
    });
    vlmResponseSub.subscribe((message) => {
        if (state.isSimMode) return;
        addBotMessage(message.data);
        dom.vlmReasoningContent.innerText = message.data;
    });
}

function connectVlmRosBridge() {
    const url = dom.rosIpInput.value.trim();
    if (!url) {
        addLog("WARNING", "VLM", "Action updated.");
        return;
    }

    try {
        state.ros = new ROSLIB.Ros({ url });
        state.ros.on("connection", () => {
            state.isVlmConnected = true;
            initRosPublishers();
            initRosSubscribers();
            addLog("SUCCESS", "VLM", `VLM ROSBridge 閻庤鐡曠亸顏嗘崲濞戙垹绠? ${url}`);
        });
        state.ros.on("error", () => {
            state.isVlmConnected = false;
            addLog("WARNING", "VLM", "Action updated.");
        });
        state.ros.on("close", () => {
            state.isVlmConnected = false;
            addLog("WARNING", "VLM", "Action updated.");
        });
    } catch (e) {
        state.isVlmConnected = false;
        addLog("WARNING", "VLM", `VLM ROSBridge 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宓椒鏉柣? ${e.message}`);
    }
}

async function fetchRobotState() {
    const response = await fetch(apiUrl("/api/state"), { cache: "no-store" });
    if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
    }
    const payload = await response.json();
    applyRobotState(payload);
    return payload;
}

function startYoloStateStream() {
    stopYoloStateStream();
    try {
        state.yoloEvents = new EventSource(apiUrl("/events"));
        state.yoloEvents.onmessage = (event) => {
            applyRobotState(JSON.parse(event.data));
        };
        state.yoloEvents.onerror = () => {
            if (state.yoloEvents) {
                state.yoloEvents.close();
                state.yoloEvents = null;
            }
            if (!state.yoloPollTimer) {
                addLog("WARNING", "YOLO", "Action updated.");
                state.yoloPollTimer = setInterval(() => {
                    fetchRobotState().catch((error) => {
                        addLog("ERROR", "YOLO", `闁哄鍎愰崰娑㈩敋?/api/state 婵犮垺鍎肩划鍓ф喆? ${error.message}`);
                    });
                }, 1000);
            }
        };
    } catch (e) {
        addLog("WARNING", "YOLO", `闂佸搫鍟版慨鐢垫兜閸洖瑙︽い鏍ㄨ壘琚?SSE 闂佺粯顭堥崺鏍焵椤戣法鍔嶇紒? ${e.message}`);
        state.yoloPollTimer = setInterval(() => {
            fetchRobotState().catch(() => {});
        }, 1000);
    }
}

function stopYoloStateStream() {
    if (state.yoloEvents) {
        state.yoloEvents.close();
        state.yoloEvents = null;
    }
    if (state.yoloPollTimer) {
        clearInterval(state.yoloPollTimer);
        state.yoloPollTimer = null;
    }
}

function applyRobotState(payload) {
    if (payload.yolo_source) {
        state.vision.yoloSource = payload.yolo_source === "aux" ? "aux" : "primary";
        updateYoloSourceUI();
    }
    updateCameraFrameStatus(payload);
    const detections = Array.isArray(payload.detections)
        ? payload.detections
        : (Array.isArray(payload.raw?.detections) ? payload.raw.detections : []);
    state.vision.rawDetections = detections;
    state.vision.imageWidth = Number(payload.raw?.image_width || payload.image_width || state.vision.imageWidth || 640);
    state.vision.imageHeight = Number(payload.raw?.image_height || payload.image_height || state.vision.imageHeight || 480);
    state.vision.detectedObjects = detections.map((item) => {
        const bbox = item.bbox || {};
        const rawLabel = item.class_name || item.label || `class ${item.class_id ?? "-"}`;
        const label = rawLabel === "book" ? "Delivery box" : rawLabel;
        const score = Number(item.score ?? item.confidence ?? item.conf ?? 0);
        const area = Number(item.area_ratio ?? 0);
        const cx = Number(bbox.cx ?? 0);
        return {
            label,
            conf: Number.isFinite(score) ? score : 0,
            dist: item.depth_m ? `${Number(item.depth_m).toFixed(2)} m` : `area:${area.toFixed(3)}`,
            pos: `cx:${Number.isFinite(cx) ? cx.toFixed(0) : "-"}`
        };
    });
    updateYoloUI();
    if (payload.last_command && payload.last_command !== state.lastRobotCommand) {
        state.lastRobotCommand = payload.last_command;
        addLog("INFO", "ROBOT", `闂佸搫顦冲▔鏇㈩敂椤掑嫬瀚夐柍褜鍓氬濠氬箣濠靛牊鐦撻柡澶屽剳闂勫嫮鈧灚纰嶇粋? ${payload.last_command}`);
    }
    if (payload.mission_state && payload.mission_state !== state.lastMissionState) {
        state.lastMissionState = payload.mission_state;
        dom.vlmReasoningContent.innerText = payload.mission_state;
    }
    applyVlmState(payload.vlm);
}

function updateCameraFrameStatus(payload) {
    if (!state.isConnected) return;

    const now = Date.now() / 1000;
    const primaryAt = Number(payload.image_frame_updated_at || 0);
    const auxAt = Number(payload.aux_image_frame_updated_at || 0);
    const primaryOk = primaryAt > 0 && now - primaryAt < 5;
    const auxOk = auxAt > 0 && now - auxAt < 5;

    dom.videoDisconnectOverlay.style.display = primaryOk ? "none" : "flex";
    dom.videoDisconnectOverlayAux.style.display = auxOk ? "none" : "flex";

    const primaryText = dom.videoDisconnectOverlay.querySelector("span");
    const auxText = dom.videoDisconnectOverlayAux.querySelector("span");
    if (primaryText) primaryText.innerText = primaryOk ? "" : "主摄无数据";
    if (auxText) auxText.innerText = auxOk ? "" : "副摄无数据";
}

async function setYoloSource(source) {
    const selected = source === "aux" ? "aux" : "primary";
    state.vision.yoloSource = selected;
    updateYoloSourceUI();

    if (!state.isConnected) {
        addLog("INFO", "YOLO", `本地切换 YOLO 画面: ${selected === "aux" ? "副摄像头" : "主摄像头"}`);
        return;
    }

    try {
        addLog("INFO", "YOLO", `正在切换 YOLO 输入到${selected === "aux" ? "副摄像头" : "主摄像头"}...`);
        const response = await fetch(apiUrl("/api/yolo/source"), {
            method: "POST",
            headers: { "Content-Type": "text/plain;charset=UTF-8" },
            body: selected,
        });
        if (!response.ok) {
            const text = await response.text();
            let errorMessage = text;
            try {
                errorMessage = JSON.parse(text).error || text;
            } catch (_) {
                // Keep raw response text.
            }
            throw new Error(errorMessage || `HTTP ${response.status}`);
        }
        const result = await response.json();
        state.vision.yoloSource = result.source === "aux" ? "aux" : "primary";
        updateYoloSourceUI();
        addLog("SUCCESS", "YOLO", `YOLO 已切换到${state.vision.yoloSource === "aux" ? "副摄像头" : "主摄像头"}。`);
        setTimeout(() => fetchRobotState().catch(() => {}), 800);
    } catch (error) {
        addLog("ERROR", "YOLO", `切换 YOLO 摄像头失败: ${error.message}`);
        fetchRobotState().catch(() => {});
    }
}

function updateYoloSourceUI() {
    const active = state.vision.yoloSource === "aux" ? "aux" : "primary";
    if (dom.yoloSourcePrimary) dom.yoloSourcePrimary.classList.toggle("active", active === "primary");
    if (dom.yoloSourceAux) dom.yoloSourceAux.classList.toggle("active", active === "aux");
    document.querySelectorAll(".camera-panel").forEach((panel) => {
        if (!panel) return;
        const isActive = panel.dataset.source === active;
        panel.classList.toggle("active-yolo", isActive);
        const tag = panel.querySelector(".camera-source-tag");
        if (tag) tag.innerText = isActive ? "YOLO" : "预览";
    });
}

function applyVlmState(vlm) {
    if (!vlm) return;
    if (state.vlm.chatMode !== "vlm") return;
    const response = vlm.last_response || "";
    if (response && response !== state.vlm.lastResponse) {
        state.vlm.lastResponse = response;
        state.vlm.isThinking = false;
        updateActiveBotMessage(response);
        dom.vlmReasoningContent.innerText = response;
        if (!vlm.waiting) {
            addLog("SUCCESS", "VLM", "Action updated.");
        }
        return;
    }
    if (vlm.waiting && !state.vlm.isThinking) {
        state.vlm.isThinking = true;
        dom.vlmReasoningContent.innerText = "閻庣懓鎲¤ぐ鍐亹閸岀偞鐒诲ù锝囶焾閻?/prompt_text闂佹寧绋戦惉濂告偤閹存繍鍤?/tts_text 闁哄鏅滈弻銊ッ?..";
    }
}

// ==========================================================================
function updateSimStepScaleButton() {
    if (!dom.simStepScaleBtn) return;
    dom.simStepScaleBtn.textContent = "1";
    dom.simStepScaleBtn.title = `sim move scale: ${state.simStepScale}`;
}

function cycleSimStepScale() {
    state.simStepScaleIndex = (state.simStepScaleIndex + 1) % CONFIG.simStepScales.length;
    state.simStepScale = CONFIG.simStepScales[state.simStepScaleIndex];
    updateSimStepScaleButton();
    addLog("INFO", "SIMULATOR", `2D sim move scale: ${state.simStepScale}`);
}

// 5. 闂佺鐭囬崘銊у幀闂佸湱顭堝ú锝夊箮閵堝鐭楅柟杈捐吂閸嬫挻鎷呯粙鎸庣槪闂?(闂佺绻掗崢褔顢欓幇鐗堝剳闁绘棃顥撻弶浠嬫煕濠婂啫顨欓悹鐐灲閹洭鎮㈤弨閫涚椤?
// ==========================================================================

function twistToDriveCommand(vx, vy, wz) {
    const axes = [
        { value: vx, command: vx >= 0 ? "(1,0,0)" : "(-1,0,0)" },
        { value: vy, command: vy >= 0 ? "(0,1,0)" : "(0,-1,0)" },
        { value: wz, command: wz >= 0 ? "(0,0,1)" : "(0,0,-1)" },
    ].filter((axis) => Math.abs(axis.value) > 0.001);

    if (axes.length === 0) return "(0,0,0)";
    axes.sort((a, b) => Math.abs(b.value) - Math.abs(a.value));
    return axes[0].command;
}

async function sendDriveCommand(command, options = {}) {
    if (!command) return;

    if (options.updateSim !== false) {
        if (state.isNavigating || state.isTaskRunning) {
            state.isNavigating = false;
            if (state.isTaskRunning) {
                pauseTaskQueue();
            }
            addLog("WARNING", "CONTROL", "手动控制已接管仿真运动。");
        }
        const values = command.replace(/[()]/g, "").split(",").map(Number);
        state.speed.vx = values[0] || 0;
        state.speed.vy = values[1] || 0;
        state.speed.wz = values[2] || 0;
        addLog("INFO", "CONTROL", `仿真小车指令: ${command}`);
    }

    try {
        const response = await fetch(apiUrl("/api/drive"), {
            method: "POST",
            headers: { "Content-Type": "text/plain;charset=UTF-8" },
            body: command,
        });
        if (!response.ok) {
            const text = await response.text();
            throw new Error(text || `HTTP ${response.status}`);
        }
        addLog("SUCCESS", "CONTROL", `实车控制已发送: ${command}`);
    } catch (error) {
        addLog("ERROR", "CONTROL", `实车控制发送失败: ${error.message}`);
    }
}

// 闂佸憡鐟﹂崹鐢电博闁垮鍎熼柡鍥╁У绾炬悂姊洪銈呮瀻閻庤濞婇獮鎰板炊閿旇棄袘 cmd_vel
function sendTwistCommand(vx, vy, wz) {
    vx = Math.max(-state.speedLimit, Math.min(state.speedLimit, vx));
    vy = Math.max(-state.speedLimit, Math.min(state.speedLimit, vy));
    wz = Math.max(-state.yawLimit, Math.min(state.yawLimit, wz));

    state.speed.vx = vx;
    state.speed.vy = vy;
    state.speed.wz = wz;

    sendDriveCommand(twistToDriveCommand(vx, vy, wz), { updateSim: false });
}

function sendNavigationGoal(x, y, yaw) {
    addLog("INFO", "NAVIGATION", `下发导航目标: X=${x.toFixed(2)}m, Y=${y.toFixed(2)}m, Yaw=${(yaw * 180 / Math.PI).toFixed(1)}°`);
    
    state.targetPose.x = x;
    state.targetPose.y = y;
    state.targetPose.yaw = yaw;
    state.isNavigating = true;
    addLog("INFO", "SIMULATOR", "本地虚拟导航启动。");

    if (state.isConnected && state.isVlmConnected && rosTopics.goalPose) {
        const qx = 0;
        const qy = 0;
        const qz = Math.sin(yaw / 2.0);
        const qw = Math.cos(yaw / 2.0);
        
        const goal = new ROSLIB.Message({
            header: {
                frame_id: 'map',
                stamp: { secs: 0, nsecs: 0 }
            },
            pose: {
                position: { x: x, y: y, z: 0.0 },
                orientation: { x: qx, y: qy, z: qz, w: qw }
            }
        });
        rosTopics.goalPose.publish(goal);
    } else if (state.isConnected) {
        addLog("WARNING", "NAVIGATION", "地瓜派已连接，但没有 ROSBridge /goal_pose 通道；仅执行本地仿真导航。");
    }
}

function sendForkliftCommand(heightPercent) {
    if (state.isConnected && state.isVlmConnected && rosTopics.forkliftCmd) {
        const msg = new ROSLIB.Message({ data: heightPercent });
        rosTopics.forkliftCmd.publish(msg);
    } else if (state.isConnected) {
        addLog("WARNING", "ACTUATOR", "地瓜派已连接，但没有 ROSBridge /forklift_cmd 通道；仅更新本地货叉仿真。");
    }
}

function cancelNavigation() {
    addLog("WARNING", "NAVIGATION", "导航任务取消。");
    state.isNavigating = false;
    state.speed.vx = 0;
    state.speed.vy = 0;
    state.speed.wz = 0;
    if (state.isConnected) {
        sendNavigationGoal(state.pose.x, state.pose.y, state.pose.yaw);
    }
}

function triggerEStop() {
    addLog("ERROR", "SAFETY", "急停触发：底盘和执行机构停止。");
    dom.estopBtn.classList.add("pulsing-active");
    setTimeout(() => {
        dom.estopBtn.classList.remove("pulsing-active");
    }, 2000);
    
    sendTwistCommand(0, 0, 0);
    cancelNavigation();
    clearTaskQueue();
}

function saveSlamMap() {
    addLog("SUCCESS", "SLAM", "发送地图保存指令：map_server/map_saver...");
    if (state.isSimMode) {
        addLog("INFO", "SIMULATOR", "仿真模式下模拟保存地图成功。");
    }
}

async function sendVlmNaturalLanguage(text) {
    try {
        state.rdkHttpBase = normalizeBaseUrl(dom.rdkHttpInput.value || state.rdkHttpBase || CONFIG.rdkHttpBase);
        state.vlm.isThinking = true;
        state.vlm.lastResponse = "";
        state.vlm.activeBotMessageEl = addBotMessage("正在等待视觉大模型返回...");
        dom.vlmReasoningContent.innerText = "正在通过 HTTP 发送到 /prompt_text，等待 /tts_text 返回...";
        const response = await fetch(apiUrl("/api/vlm/prompt"), {
            method: "POST",
            headers: { "Content-Type": "text/plain;charset=UTF-8" },
            body: text,
        });
        if (!response.ok) {
            const errorText = await response.text();
            throw new Error(errorText || `HTTP ${response.status}`);
        }
        addLog("SUCCESS", "VLM", `已发送视觉大模型指令到 /prompt_text: ${text}`);
        setTimeout(() => fetchRobotState().catch(() => {}), 300);
    } catch (error) {
        state.vlm.isThinking = false;
        addBotMessage(`视觉大模型发送失败：${error.message}`);
        addLog("ERROR", "VLM", `视觉大模型发送失败: ${error.message}`);
    }
}

async function sendModelNaturalLanguage(text) {
    try {
        state.rdkHttpBase = normalizeBaseUrl(dom.rdkHttpInput.value || state.rdkHttpBase || CONFIG.rdkHttpBase);
        state.vlm.isThinking = true;
        state.vlm.lastResponse = "";
        state.vlm.activeBotMessageEl = addBotMessage("正在等待大模型返回...");
        dom.vlmReasoningContent.innerText = "正在发送到板端大模型，请稍候...";
        const response = await fetch(apiUrl("/api/model/prompt"), {
            method: "POST",
            headers: { "Content-Type": "text/plain;charset=UTF-8" },
            body: text,
        });
        const payload = await response.json().catch(() => null);
        if (!response.ok || !payload || payload.ok === false) {
            throw new Error(payload?.error || payload?.response || `HTTP ${response.status}`);
        }
        const reply = payload.response || "大模型已收到指令。";
        state.vlm.lastResponse = reply;
        state.vlm.isThinking = false;
        updateActiveBotMessage(reply);
        dom.vlmReasoningContent.innerText = reply;
        addLog("SUCCESS", "MODEL", `大模型返回 (${payload.transport || "unknown"}): ${reply}`);
        setTimeout(() => fetchRobotState().catch(() => {}), 300);
    } catch (error) {
        state.vlm.isThinking = false;
        updateActiveBotMessage(`大模型发送失败：${error.message}`);
        dom.vlmReasoningContent.innerText = `大模型发送失败：${error.message}`;
        addLog("ERROR", "MODEL", `大模型发送失败: ${error.message}`);
    }
}

// ==========================================================================
// 6. 闂佸綊娼ч鍛叏閳哄懏鐒奸柕澶涚畱娴犳﹢姊洪锝嗩潡缂?(闂備焦顑欓崰姘?WASD & 闂佹儳绻戠喊宥団偓姘懇楠炴宕堕妸锔剧С nipplejs)
// ==========================================================================
function initKeyboard() {
    window.addEventListener("keydown", (e) => {
        if (document.activeElement.tagName === "INPUT" || document.activeElement.tagName === "TEXTAREA") {
            return;
        }
        
        switch (e.key.toLowerCase()) {
            case "w": state.keys.w = true; break;
            case "s": state.keys.s = true; break;
            case "a": state.keys.a = true; break;
            case "d": state.keys.d = true; break;
            case "q": state.keys.q = true; break;
            case "e": state.keys.e = true; break;
        }
        updateKeyboardSpeed();
    });

    window.addEventListener("keyup", (e) => {
        switch (e.key.toLowerCase()) {
            case "w": state.keys.w = false; break;
            case "s": state.keys.s = false; break;
            case "a": state.keys.a = false; break;
            case "d": state.keys.d = false; break;
            case "q": state.keys.q = false; break;
            case "e": state.keys.e = false; break;
        }
        updateKeyboardSpeed();
    });
}

function updateKeyboardSpeed() {
    let vx = 0;
    let vy = 0;
    let wz = 0;
    
    // 闂佸憡鎸哥粔鎾箖?
    if (state.keys.w) vx = state.speedLimit;
    if (state.keys.s) vx = -state.speedLimit;
    
    // 閻庡綊娼荤粻鎴ｃ亹缁嬭鐔碱敂閸?
    if (state.keys.a) vy = state.speedLimit;
    if (state.keys.d) vy = -state.speedLimit;
    
    // 閻庡綊娼荤粻鎴ｃ亹閹间礁鍌ㄩ柣鏃堟敱閸曢箖鏌ゆ總澶夋捣婵?
    if (state.keys.q) wz = state.yawLimit;
    if (state.keys.e) wz = -state.yawLimit;
    
    if (vx !== 0 || vy !== 0 || wz !== 0) {
        // 闂佸綊娼ч鍛叏閳哄懎绠肩€广儱娲﹀Λ璇裁归崗鑲╊暡濠⒀嶇畵瀵剟顢橀垾铏唶婵炴垶鎹侀褔顢氶柆宥嗗殝妞ゅ繐瀚€氭彃霉閻樹警鍤欏┑顔惧枛濮婂ジ鎮㈠畡鎵粴
        if (state.isNavigating || state.isTaskRunning) {
            state.isNavigating = false;
            pauseTaskQueue();
            addLog("WARNING", "CONTROL", "Action updated.");
        }
    }
    
    sendTwistCommand(vx, vy, wz);
}

function initJoystick() {
    const options = {
        zone: dom.joystickZone,
        mode: "static",
        position: { left: "50%", top: "50%" },
        color: "cyan",
        size: 90
    };
    
    const manager = nipplejs.create(options);
    
    manager.on("move", (evt, data) => {
        if (!data || !data.vector) return;
        
        // 婵°倗娅㈢粻鎴﹀储閻樼數妫柛鎰▕濞兼帡寮堕悜鍡楁珯缂佽鲸澹嗘禍鎼佸幢濡警鏉归梺鍝勮嫰鐎氼剙顭囧Δ鍛唨闁搞儺浜濈粊顕€鏌涢弽銊уⅶ閻犳劗鍠撻惀顏堝锤濡炶浜鹃柣鏂垮槻椤?vx 闂佸憡绮岄張顒勬惎缂備礁顦…鐑藉焵椤掑倸鏋庨悗?vy
        // data.vector.y 婵炲濯寸紞鈧柕鍡楀暣瀹曟粌顓奸崨顔尖偓鍨叏閿濆棙鐓ｇ紒韬插劦閺佸秶鈧濡簍a.vector.x 婵炲濯寸紞鈧柕鍡楀暙椤斿繘鏁冮埀顒冦亹缁嬪晝鎺楀棘閸撗傜矗
        const vx = data.vector.y * state.speedLimit;
        const vy = -data.vector.x * state.speedLimit; // 闂佸綊娼ч鍡涙倿濞差亜瑙﹂柟瀛樼箓缁€渚€鏌ㄥ☉妯肩劮婵犙冩噺濞煎繘宕卞Δ浣糕偓濠氭煕濞嗘帗鐝柧鍡欑磼?        
        // 闂佸搫鍟鍫澝归崱娑欑劵闁绘柨鍢查濠囨煕閿斿搫濡藉褎绮撳濠氬礋椤掆偓閻﹁鈽夐幘鎰佸剱婵＄偛鍊圭粙?闂佹寧绋戦惌鍌炲极椤撱垺鍎庢俊顖氭惈鐠佹煡寮堕崼婵囧櫣濠殿噮浜顔锯偓锝庡幗缁?
        if (state.isNavigating || state.isTaskRunning) {
            state.isNavigating = false;
            pauseTaskQueue();
            addLog("WARNING", "CONTROL", "Action updated.");
        }
        
        sendTwistCommand(vx, vy, 0.0);
    });
    
    manager.on("end", () => {
        sendTwistCommand(0.0, 0.0, 0.0);
    });
}

// ==========================================================================
// 7. SLAM 2D 婵炲瓨鍤庨崐鎾惰姳娴兼潙鎹堕柡澶嬪缁?Canvas 缂傚倷鐒﹂敋闁糕晜顨嗙粙澶嬫償閵忊剝鍎ラ梺?// ==========================================================================
let mapCtx = null;

// 濠碘槅鍨崜婵堚偓姘懇閹啴宕熼鍡樺皾闁圭厧鐡ㄩ幐绋匡耿閹绢喖鐐婇柛鎾楀啰鐣炬繝鈷€鍐ㄦ殭闁告ɑ鎸惧Σ?(闂佹椿娼块崝瀣姳椤掍胶顩烽悹鎭掑妽閸?
const simEnvironment = {
    walls: [
        [-10, -10, 20, 0.3],
        [-10, 10, 20, 0.3],
        [-10, -10, 0.3, 20],
        [10, -10, 0.3, 20],
        [-5, -4, 4, 1.5],
        [3, -4, 3, 1.5],
        [-5, 3, 4, 1.5],
        [3, 3, 3, 1.5],
    ],
    zones: [
        { name: "充电区", x: -8, y: -8, color: "rgba(0, 230, 118, 0.15)", border: "var(--green)" },
        { name: "仓库 A", x: -6, y: -1, color: "rgba(0, 242, 254, 0.12)", border: "var(--cyan)" },
        { name: "仓库 B", x: 5, y: -1, color: "rgba(185, 39, 252, 0.12)", border: "var(--purple)" },
        { name: "卸货点 C", x: -8, y: 7, color: "rgba(255, 145, 0, 0.12)", border: "var(--orange)" }
    ],
    simPallet: { x: -6.0, y: -1.0, width: 0.8, height: 0.8, color: "rgba(255, 214, 0, 0.5)", loaded: false }
};

function initMapCanvas() {
    if (!dom.mapCanvas) {
        addLog("ERROR", "MAP", "地图画布未找到。");
        return;
    }
    mapCtx = dom.mapCanvas.getContext("2d");
    resizeCanvas();
    window.addEventListener("resize", resizeCanvas);
    resetMapView();

    dom.mapCanvas.addEventListener("mousedown", (e) => {
        state.map.isDragging = true;
        state.map.dragStartX = e.clientX - state.map.offsetX;
        state.map.dragStartY = e.clientY - state.map.offsetY;
    });

    window.addEventListener("mousemove", (e) => {
        if (!state.map.isDragging) return;
        state.map.offsetX = e.clientX - state.map.dragStartX;
        state.map.offsetY = e.clientY - state.map.dragStartY;
        drawMap();
    });

    window.addEventListener("mouseup", () => {
        state.map.isDragging = false;
    });

    dom.mapCanvas.addEventListener("wheel", (e) => {
        e.preventDefault();
        const zoomFactor = 1.15;
        const rect = dom.mapCanvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const worldX = (mouseX - state.map.offsetX) / state.map.scale;
        const worldY = (mouseY - state.map.offsetY) / state.map.scale;

        state.map.scale *= e.deltaY < 0 ? zoomFactor : 1 / zoomFactor;
        state.map.scale = Math.max(1, Math.min(30, state.map.scale));
        state.map.offsetX = mouseX - worldX * state.map.scale;
        state.map.offsetY = mouseY - worldY * state.map.scale;
        drawMap();
    });

    dom.mapCanvas.addEventListener("dblclick", (e) => {
        const rect = dom.mapCanvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const canvasX = mouseX - state.map.offsetX;
        const canvasY = mouseY - state.map.offsetY;
        const px = (canvasX - dom.mapCanvas.width / 2) / state.map.scale / 10;
        const py = -(canvasY - dom.mapCanvas.height / 2) / state.map.scale / 10;
        const yaw = Math.atan2(py - state.pose.y, px - state.pose.x);

        dom.targetX.value = px.toFixed(2);
        dom.targetY.value = py.toFixed(2);
        dom.targetYaw.value = (yaw * 180 / Math.PI).toFixed(0);
        sendNavigationGoal(px, py, yaw);
    });
}
function resizeCanvas() {
    const parent = dom.mapCanvas.parentElement;
    dom.mapCanvas.width = Math.max(parent.clientWidth, 640);
    dom.mapCanvas.height = Math.max(parent.clientHeight, 360);
    drawMap();
}

function resetMapView() {
    state.map.scale = CONFIG.mapScaleDefault;
    state.map.offsetX = 0;
    state.map.offsetY = 0;
    drawMap();
}

// 闂佺鍕闁绘牭绲惧顏堫敆娴ｇ绨ラ柡澶婄墕閹冲孩鎱ㄩ鍫熸櫖婵﹩鍋嗛悷顖炴煟閿濆懐鐏卞褍娼￠幃鍫曞幢濡や胶绋勯梺?(m) => Canvas闂佹眹鍨奸褏绮╅悜钘夌＝闊洦娲滈ˇ閬嶆煕瑜庨崝鏍偉?(px)
function worldToCanvas(wx, wy) {
    const cx = dom.mapCanvas.width / 2 + wx * 10 * state.map.scale + state.map.offsetX;
    const cy = dom.mapCanvas.height / 2 - wy * 10 * state.map.scale + state.map.offsetY;
    return { x: cx, y: cy };
}

// 闂佸搫绉堕…鍫㈢紦妤ｅ啫鎹堕柡澶嬪缁傚牏绱撴担濮戭亪宕哄Δ鈧锝夊即閻斿憡鍎?
function drawMap() {
    if (!mapCtx) return;
    
    // 1. 濠电偞鎸搁幊鎰板煘閺嶎厽鍋ㄩ悹鍥皺椤?
    mapCtx.fillStyle = "#04060b";
    mapCtx.fillRect(0, 0, dom.mapCanvas.width, dom.mapCanvas.height);
    
    // 2. 缂傚倷鐒﹂敋闁糕晜顨嗙粋鎺旀崉閵婏箑鐒哥紓鍌氬暞閸ㄥ爼鎮ч幖浣瑰殑閻忕偟鍋撻悵?(闁圭厧鐡ㄥú婊兠鸿箛鏇犵煋閻犲洦褰冭闂佸憡鐟ラ崐濠氬矗?
    const gridSpacing = 2.0; // 濠?缂備緡鍋勭壕顓犲垝椤栫偛绀嗛柡鍫㈡暩椤忛亶鏌℃径娑氱？闁?
    mapCtx.strokeStyle = "rgba(255, 255, 255, 0.02)";
    mapCtx.lineWidth = 1;
    
    for (let x = -15; x <= 15; x += gridSpacing) {
        const start = worldToCanvas(x, -15);
        const end = worldToCanvas(x, 15);
        mapCtx.beginPath();
        mapCtx.moveTo(start.x, start.y);
        mapCtx.lineTo(end.x, end.y);
        mapCtx.stroke();
    }
    for (let y = -15; y <= 15; y += gridSpacing) {
        const start = worldToCanvas(-15, y);
        const end = worldToCanvas(15, y);
        mapCtx.beginPath();
        mapCtx.moveTo(start.x, start.y);
        mapCtx.lineTo(end.x, end.y);
        mapCtx.stroke();
    }
    
    // 3. 缂傚倷鐒﹂敋闁糕晜顨婂畷鐘诲传閸曨厼骞嶉梺鍝勭Т濞层劑顢?(Zone)
    simEnvironment.zones.forEach(zone => {
        const pos = worldToCanvas(zone.x, zone.y);
        const size = 3 * 10 * state.map.scale; // 3x3缂備緡鍋勯崯璺ㄤ焊椤栫偛鏄?        
        mapCtx.fillStyle = zone.color;
        mapCtx.strokeStyle = zone.border;
        mapCtx.lineWidth = 1;
        mapCtx.beginPath();
        mapCtx.rect(pos.x - size/2, pos.y - size/2, size, size);
        mapCtx.fill();
        mapCtx.stroke();
        
        // 闂佸憡鐗曢幖顐︽偂濞嗘挸妫橀柛銉戝懏鎲?
        mapCtx.fillStyle = "rgba(255, 255, 255, 0.4)";
        mapCtx.font = `${Math.max(9, 3 * state.map.scale)}px sans-serif`;
        mapCtx.textAlign = "center";
        mapCtx.textBaseline = "middle";
        mapCtx.fillText(zone.name, pos.x, pos.y);
    });

    // 4. 缂傚倷鐒﹂敋闁糕晜顨堥幏纭咁槾濠⒀冩健楠炲秴螣閸忚偐锛?(Pallet)
    if (state.isSimMode && !simEnvironment.simPallet.loaded) {
        const p = simEnvironment.simPallet;
        const pos = worldToCanvas(p.x, p.y);
        const w = p.width * 10 * state.map.scale;
        const h = p.height * 10 * state.map.scale;
        mapCtx.fillStyle = p.color;
        mapCtx.strokeStyle = "#ffd600";
        mapCtx.lineWidth = 2;
        mapCtx.beginPath();
        mapCtx.rect(pos.x - w/2, pos.y - h/2, w, h);
        mapCtx.fill();
        mapCtx.stroke();
        
        mapCtx.beginPath();
        mapCtx.moveTo(pos.x - w/4, pos.y - h/2);
        mapCtx.lineTo(pos.x - w/4, pos.y + h/2);
        mapCtx.moveTo(pos.x + w/4, pos.y - h/2);
        mapCtx.lineTo(pos.x + w/4, pos.y + h/2);
        mapCtx.stroke();
    }
    
    // 5. 缂傚倷鐒﹂敋闁糕晜顨婂鎯ь煥閸滀焦娈归梺缁樸仜閺呮盯銆傞悙顒佸?(Wall)
    mapCtx.fillStyle = "rgba(100, 116, 139, 0.8)";
    mapCtx.strokeStyle = "rgba(148, 163, 184, 0.5)";
    mapCtx.lineWidth = 2;
    
    simEnvironment.walls.forEach(wall => {
        const pos = worldToCanvas(wall[0], wall[1]);
        const w = wall[2] * 10 * state.map.scale;
        const h = -wall[3] * 10 * state.map.scale; // Y闁哄鍋炲娆忣嚕娴犲瑙?        
        mapCtx.beginPath();
        mapCtx.rect(pos.x, pos.y, w, h);
        mapCtx.fill();
        mapCtx.stroke();
    });
    
    // 6. 缂傚倷鐒﹂敋闁糕晜顨婇幊娑㈩敂閸愩劎妲ｉ柣搴濈祷婢瑰牓宕洪崨顖涘磯妞ゆ牗姘ㄧ粣鐐烘偡濞嗗繐顏╅柛銊︾箘閻?(Path)
    if (state.isNavigating) {
        const startPos = worldToCanvas(state.pose.x, state.pose.y);
        const endPos = worldToCanvas(state.targetPose.x, state.targetPose.y);
        
        mapCtx.strokeStyle = "rgba(185, 39, 252, 0.6)"; // 缂備線纭搁崑澶嬬珶婵犲洤鐭楅柟瀛樼箓鐢劑鎮规笟顖氱仩缂?
        mapCtx.shadowColor = "rgba(185, 39, 252, 0.8)";
        mapCtx.shadowBlur = 8;
        mapCtx.lineWidth = 3;
        
        mapCtx.beginPath();
        mapCtx.moveTo(startPos.x, startPos.y);
        
        // 闁荤姳绶ょ槐鏇㈡偩閼姐倗涓嶉柍褜鍓熷畷锟犲即閻愬灚鎷遍梻渚囧枔閸斿秵鎱ㄥ畝鈧惀?(濠碘槅鍨崜婵堚偓姘懅娴狅箓鍩€椤掑嫭鐒鹃柕濠庣厛濞兼劙鏌涢幒鎾崇瑨婵?
        if (Math.abs(state.pose.x - state.targetPose.x) > 2 && Math.abs(state.pose.y - state.targetPose.y) > 2) {
            const midX = (state.pose.x + state.targetPose.x) / 2;
            const midY = state.pose.y > 0 ? 5.5 : -5.5; // 闁荤喐鐟ョ€氼剟宕瑰┑鍫㈢＜闁哄洠妲呴弨?
            const midPos = worldToCanvas(midX, midY);
            mapCtx.lineTo(midPos.x, midPos.y);
        }
        
        mapCtx.lineTo(endPos.x, endPos.y);
        mapCtx.stroke();
        
        mapCtx.shadowBlur = 0;
        
        mapCtx.fillStyle = "var(--red)";
        mapCtx.beginPath();
        mapCtx.arc(endPos.x, endPos.y, 6, 0, 2*Math.PI);
        mapCtx.fill();
        
        // 闂佺儵鏅╅崰妤呮偉閿濆妫橀柣鐔哄閸婅崵绱掗悪鈧崢濂稿Φ?
        mapCtx.strokeStyle = "white";
        mapCtx.lineWidth = 2;
        mapCtx.beginPath();
        mapCtx.moveTo(endPos.x, endPos.y);
        mapCtx.lineTo(
            endPos.x + Math.cos(state.targetPose.yaw) * 12,
            endPos.y - Math.sin(state.targetPose.yaw) * 12
        );
        mapCtx.stroke();
    }
    
    // 7. 缂傚倷鐒﹂敋闁糕晜顨呴埞鎴﹀焵椤掑嫬绀傚鑸靛姈缁侇偊寮堕崼銏犱壕濠㈢懓鍊块獮鎾圭疀閺囩偞鐤囬梺?(Lidar Scan)
    if (state.isSimMode) {
        mapCtx.strokeStyle = "rgba(0, 230, 118, 0.15)";
        mapCtx.lineWidth = 1;
        const scanLines = 60;
        const maxRange = 6.0;
        const rPos = worldToCanvas(state.pose.x, state.pose.y);

        for (let i = 0; i < scanLines; i++) {
            const angle = state.pose.yaw + (i * 2 * Math.PI / scanLines);
            const dist = maxRange * (0.8 + Math.random() * 0.2);
            const scanPos = worldToCanvas(
                state.pose.x + Math.cos(angle) * dist,
                state.pose.y + Math.sin(angle) * dist
            );
            
            mapCtx.beginPath();
            mapCtx.moveTo(rPos.x, rPos.y);
            mapCtx.lineTo(scanPos.x, scanPos.y);
            mapCtx.stroke();
        }
    }
    
    // 8. 缂傚倷鐒﹂敋闁糕晜顨婂鐢稿传閸曨剛褰滄繛瀛樼矌閸庛倕锕㈤悧鍫熷?(Mecanum Robot)
    const robPos = worldToCanvas(state.pose.x, state.pose.y);
    const robSize = 1.6 * 10 * state.map.scale; // 濠殿噯绲鹃弻褏娆㈤妷锕€绶炵憸宥夋儍?
    
    mapCtx.save();
    mapCtx.translate(robPos.x, robPos.y);
        mapCtx.rotate(-state.pose.yaw); // Canvas Y is inverted relative to world coordinates.
    
    // 闁圭厧鐡ㄥú婊兠鸿箛娑欏殑閻忕偟鍋撻悵?- 闂備椒鍗抽ˉ鎾诲磻閹烘顥堥柟鐑樻礀椤ュ繘鏌涘Δ鈧敃顏堝焵椤掆偓缁绘垵危?
    mapCtx.fillStyle = "rgba(0, 242, 254, 0.2)";
    mapCtx.strokeStyle = "var(--cyan)";
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    mapCtx.rect(-robSize/2, -robSize/2, robSize, robSize);
    mapCtx.fill();
    mapCtx.stroke();
    
    // 婵°倗娅㈢粻鎴﹀储閻樼數妫柛鎰▕濞兼帡寮?(闂佹悶鍎茬粙鎰版煂濠婂嫭濮滄い鏃囧亹閹?
    mapCtx.fillStyle = "#1e293b";
    mapCtx.strokeStyle = "rgba(255,255,255,0.3)";
    mapCtx.lineWidth = 1;
    
    const wheelW = robSize / 3.5;
    const wheelH = robSize / 1.8;
    
    // LF, RF, LB, RB
    mapCtx.fillRect(-robSize/2 - wheelW/2, -robSize/2, wheelW, wheelH);
    mapCtx.strokeRect(-robSize/2 - wheelW/2, -robSize/2, wheelW, wheelH);
    
    mapCtx.fillRect(robSize/2 - wheelW/2, -robSize/2, wheelW, wheelH);
    mapCtx.strokeRect(robSize/2 - wheelW/2, -robSize/2, wheelW, wheelH);
    
    mapCtx.fillRect(-robSize/2 - wheelW/2, robSize/2 - wheelH, wheelW, wheelH);
    mapCtx.strokeRect(-robSize/2 - wheelW/2, robSize/2 - wheelH, wheelW, wheelH);
    
    mapCtx.fillRect(robSize/2 - wheelW/2, robSize/2 - wheelH, wheelW, wheelH);
    mapCtx.strokeRect(robSize/2 - wheelW/2, robSize/2 - wheelH, wheelW, wheelH);
    
    // 闂佸憡鎸哥粔鍫曨敂椤掑嫬鐭楀鑸靛姇閳诲繘鏌￠崼銏犲妺闁?(婵炵鍋愰…鍫濓耿椤忓牆绀堢€广儱妫欓悡?- 婵炴垶鎸荤划宥夊蓟閻斿摜鈻斿Δ锔藉缁ㄧ姴顭?
    mapCtx.strokeStyle = "#e2e8f0";
    mapCtx.lineWidth = 3;
    const forkLen = robSize / 1.5;
    const forkGap = robSize / 3;
    
    mapCtx.beginPath();
    mapCtx.moveTo(robSize/2, -forkGap);
    mapCtx.lineTo(robSize/2 + forkLen, -forkGap);
    mapCtx.moveTo(robSize/2, forkGap);
    mapCtx.lineTo(robSize/2 + forkLen, forkGap);
    mapCtx.stroke();
    
    // 闁荤姵鍓崘銊ч獓婵炴垶鎸搁敃銈呯暦鐏炵虎娈楁俊顖滅帛椤ρ囨煟閵娿儱顏悗鍨皑缁牓鎼归悷鐗堢枃 (闂佽　鍋撻柣鐔告緲缂嶄礁霉濠婂骸鏋涢悗纭呮珪閿涙劕顭ㄩ崱妤婃)
    if (state.forklift.height > 10) {
        mapCtx.strokeStyle = "rgba(0, 242, 254, 0.8)";
        mapCtx.lineWidth = 4;
        mapCtx.beginPath();
        mapCtx.moveTo(robSize/2 + 2, -forkGap);
        mapCtx.lineTo(robSize/2 + forkLen - 2, -forkGap);
        mapCtx.moveTo(robSize/2 + 2, forkGap);
        mapCtx.lineTo(robSize/2 + forkLen - 2, forkGap);
        mapCtx.stroke();
    }
    
    // 婵犵鈧啿鈧綊鎮樻径灞惧鐟滃秴顕ｉ懜鐢碘枖濠电姴鎳忕粊浼存煛閸繍妲风紒顭掔節楠炲秴螣閸忚偐锛橀梺鎸庣☉閼活垶鎯冮姀銈呯婵☆垰鍚嬬壕鎼佹煟閵忥綆鍤欐繝鈧鍫濈煑濠㈣泛瀛╃花鐘差潡鐠囧眰浠掔紒?
    if (state.forklift.payload > 0) {
        const pw = 0.8 * 10 * state.map.scale;
        mapCtx.fillStyle = "rgba(255, 214, 0, 0.7)";
        mapCtx.strokeStyle = "#ffd600";
        mapCtx.lineWidth = 1.5;
        mapCtx.beginPath();
        mapCtx.rect(robSize/2 + forkLen/2 - pw/2, -pw/2, pw, pw);
        mapCtx.fill();
        mapCtx.stroke();
    }

    mapCtx.fillStyle = "var(--cyan)";
    mapCtx.beginPath();
    mapCtx.moveTo(robSize/4, 0);
    mapCtx.lineTo(-robSize/4, -robSize/4);
    mapCtx.lineTo(-robSize/6, 0);
    mapCtx.lineTo(-robSize/4, robSize/4);
    mapCtx.closePath();
    mapCtx.fill();
    
    mapCtx.restore();
}

// ==========================================================================
// 8. YOLO 闁荤喐鐟ュΛ婊堬綖鎼淬劍鍎庨柟瀵稿仧閵堟挳鏌ｉ姀鐙€鐓兼繛?Canvas 濠电偞鎸稿鍫曟偂?(婵炲濮惧▔鏇烇耿?3D 濠电儑绲藉畷顒傗偓瑙勫▕閺屽懓绠涘鍏肩秺闂佹眹鍨归…宄邦焽?
// ==========================================================================
let visionCtx = null;
let visionCtxAux = null;

function updateVisionCanvas() {
    prepareVisionCanvas(dom.visionCanvas);
    prepareVisionCanvas(dom.visionCanvasAux);
    visionCtx = dom.visionCanvas.getContext("2d");
    visionCtxAux = dom.visionCanvasAux.getContext("2d");
    
    if (state.isConnected) {
        drawRealYoloOverlay();
    } else {
        drawSimVisionFeed();
    }
}

function prepareVisionCanvas(canvas) {
    if (!canvas) return;
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
}

function getDetectionBbox(bbox) {
    const w = Number(bbox.w ?? bbox.width ?? 0);
    const h = Number(bbox.h ?? bbox.height ?? 0);
    let x1 = Number(bbox.x1 ?? bbox.left);
    let y1 = Number(bbox.y1 ?? bbox.top);

    if (!Number.isFinite(x1)) {
        x1 = Number(bbox.x ?? bbox.cx ?? 0) - w / 2;
    }
    if (!Number.isFinite(y1)) {
        y1 = Number(bbox.y ?? bbox.cy ?? 0) - h / 2;
    }

    return { x1, y1, w, h };
}

function drawRealYoloOverlay() {
    const activeCanvas = state.vision.yoloSource === "aux" ? dom.visionCanvasAux : dom.visionCanvas;
    const activeCtx = state.vision.yoloSource === "aux" ? visionCtxAux : visionCtx;
    if (!activeCanvas || !activeCtx) return;

    const canvas = activeCanvas;
    const cw = canvas.width;
    const ch = canvas.height;
    activeCtx.clearRect(0, 0, cw, ch);

    const iw = Math.max(1, Number(state.vision.imageWidth || 640));
    const ih = Math.max(1, Number(state.vision.imageHeight || 480));
    const scale = Math.min(cw / iw, ch / ih);
    const ox = (cw - iw * scale) / 2;
    const oy = (ch - ih * scale) / 2;

    const detections = Array.isArray(state.vision.rawDetections) ? state.vision.rawDetections : [];
    detections.forEach((item) => {
        const bbox = getDetectionBbox(item.bbox || {});
        if (!Number.isFinite(bbox.w) || !Number.isFinite(bbox.h) || bbox.w <= 0 || bbox.h <= 0) {
            return;
        }

        const x = ox + bbox.x1 * scale;
        // The live image is flipped vertically with CSS, so flip detection boxes too.
        const y = oy + (ih - bbox.y1 - bbox.h) * scale;
        const bw = bbox.w * scale;
        const bh = bbox.h * scale;
        const label = item.class_name || item.label || `class ${item.class_id ?? "-"}`;
        const score = Number(item.score ?? item.confidence ?? item.conf ?? 0);
        const color = label.toLowerCase().includes("person") ? "#ff9100" : "#00f2fe";
        const text = `${label} ${Number.isFinite(score) ? Math.round(score * 100) : 0}%`;

        activeCtx.save();
        activeCtx.strokeStyle = color;
        activeCtx.lineWidth = 3;
        activeCtx.shadowColor = color;
        activeCtx.shadowBlur = 8;
        activeCtx.strokeRect(x, y, bw, bh);
        activeCtx.shadowBlur = 0;

        activeCtx.font = "bold 13px sans-serif";
        const textWidth = activeCtx.measureText(text).width;
        const labelY = Math.max(4, y - 22);
        activeCtx.fillStyle = "rgba(3, 4, 7, 0.82)";
        activeCtx.fillRect(x, labelY, textWidth + 12, 20);
        activeCtx.fillStyle = color;
        activeCtx.fillText(text, x + 6, labelY + 14);
        activeCtx.restore();
    });
}

function drawSimVisionFeed() {
    if (!visionCtx) return;
    
    const w = dom.visionCanvas.width;
    const h = dom.visionCanvas.height;
    
    // 1. 缂傚倷鐒﹂敋闁糕晜顨婇幊妤冧沪閻愵剛褰查梺鎸庣⊕閻喚鍒掗妸锔藉劅闁圭偓鍓氶弳浼存⒒閸偅绶氱紒妤€鐭傚畷鐑藉箥椤旀儳鐓㈤梻渚囧亜缁绘锝炵€ｎ剛妫柟绋垮閽?
    visionCtx.fillStyle = "#0b0f19";
    visionCtx.fillRect(0, 0, w, h);
    
    // 婵犮垹鐏堥弲鐐存叏閹惰棄绾?闂侀潻缂氶崡鍐差焽娴兼潙绀嗛柛鈩冪懄濞呮洜绱?(闂侀潻闄勫姗€鏌﹂埡鍐／?
    const horizon = h / 2.2;
    visionCtx.fillStyle = "#06080d";
    visionCtx.fillRect(0, 0, w, horizon);
    
    visionCtx.strokeStyle = "rgba(255,255,255,0.015)";
    visionCtx.lineWidth = 1;
    for (let i = -w; i <= w * 2; i += 80) {
        visionCtx.beginPath();
        visionCtx.moveTo(w/2, 0);
        visionCtx.lineTo(i, horizon);
        visionCtx.stroke();
    }
    
    visionCtx.strokeStyle = "rgba(255, 255, 255, 0.03)";
    for (let i = -w; i <= w * 2; i += 60) {
        visionCtx.beginPath();
        visionCtx.moveTo(w/2, horizon);
        visionCtx.lineTo(i, h);
        visionCtx.stroke();
    }
    
    for (let i = horizon; i < h; i += (h - horizon) / 6) {
        visionCtx.beginPath();
        visionCtx.moveTo(0, i);
        visionCtx.lineTo(w, i);
        visionCtx.stroke();
    }
    
    // 2. 闂佸搫绉烽～澶婄暤娓氣偓瀵敻宕崟顒傚綔婵炲瓨绮忓銊х箔瀹€鍕婵☆垰鍚嬬壕鎼佹煟閵娿儱顏紒鎰〒缁寮拌箛锝呮倎缂備胶濮甸〃鍡樻櫠椤撱垺鍎庢俊顖滃劋闊剟鏌ｉ姀鐙€鐓兼繛鑲╁缁嬪绻濋崟顒傛殸闂佺鍩栧ú鏍ㄧ附?
    // 闂佺懓鐏氶…鍥鸿箛娑樻嵍闁靛ě鍐ｆ寘闂佽崵鍋涘Λ鏃傜箔濮椻偓閹儳鐣濋埀顒€鈻撻幋鐐碘枖闁哄稁鍋呭▍鏇㈡煕瑜庨崝鏍偉? (-6.0, -1.0)
    const dx = simEnvironment.simPallet.x - state.pose.x;
    const dy = simEnvironment.simPallet.y - state.pose.y;
    
    // 闁诲繐绻愬Λ婊呯玻濞戞氨鐭夌紓浣姑～宀€鎲搁弶鍨埞闁糕晛鐭傚鐢稿传閸曨剛褰滄繛瀛樼矎濞咃絽鈻撻幋锔藉剮闁告稑锕ョ花姘舵煕瑜庨崝鏍偉閿濆洤瀵?(闂佸搫鐗嗛幖顐⑩枍閹烘挾顩查柣鎴炆戠弧鍌炴煕濮橆厼鐏ｉ悹鎰墯)
    // 闂佸搫鍟鍫澝归崱娑樼煑婵☆垵顕ф惔濠囨煟椤撴稒娅婃俊顖楀亾
    const cameraX = dx * Math.cos(state.pose.yaw) + dy * Math.sin(state.pose.yaw);
    const cameraY = -dx * Math.sin(state.pose.yaw) + dy * Math.cos(state.pose.yaw);
    
    state.vision.detectedObjects = []; // 濠殿噯绲界换鎴︻敃閻撳簺鈧帡宕ㄩ妤佹櫈闂備焦褰冪粔鍫曟偪?
    
    // 闂佺懓鐏氶…鍥鸿箛娑樻嵍闁靛ě鍕偛闂佸搫鐗嗛幖顐ｆ櫠閻樿妫橀柣褍鎽滅粈澶愭煙闂堟稓孝鐟滄媽灏欓幃顕€顢曢妶鍡樻瘞闂侀潻璐熼崝搴ㄥ极妤ｅ啫绾у鑸电摃閸?
    if (cameraX > 0.3 && !simEnvironment.simPallet.loaded) {
        // 闁荤姳绶ょ槐鏇㈡偩婵犳碍鍎庨柛娑橈攻缁ㄦ岸鎮峰▎蹇旑棞婵犙€鍋撻柣鐔哥懄婢х枾V
        // cameraX婵炴垶鎸鹃崕銈囨崲娴ｈ鍎熼柨鏃囧亹缁€濉゛meraY婵炴垶鎸搁幖顐ｇ閸濄儳鐭夐悹浣哥枃閸橆剟鐓崶褍鏆熸繛鍫熷灥铻ｆい蹇撳閸婂鎮归悜妯肩畼妞?
        // 缂備胶濮崑鎾绘煕濡や焦绀堟繛鍫熷灴閺屽懓绠涘鍏肩秺闂佺鍩栧ú鏍ㄧ附? 闂佺绉寸换鎺旂矆鐎靛牓鏌涚€ｎ亞绠虫い?= (cameraY / cameraX) * 闂佺粯甯粻鎺旂玻濞戙垺鏅悘鐐舵閸撹偐绱掓潏鈺佇㈤柡鍕€婚埀?= 闁诲骸婀遍崑銈咁瀶椤栨粈鐒婇柛婵嗗閸?/ cameraX * 缂傚倸鍊甸弲婊堝棘?
        const focalLength = w * 0.8;
        const projX = w / 2 - (cameraY / cameraX) * focalLength;
        const projY = horizon + (0.3 / cameraX) * focalLength; // 婵°倕鍊归敋閻庤濞婂畷鎴ｇ疀閺囥劎鈧?
        
        const sizeX = (0.8 / cameraX) * focalLength; // 0.8缂備緡鍋勯崯鍧楊敊?
        const sizeY = (0.4 / cameraX) * focalLength; // 0.4缂備緡鍋呮繛濠囨偟?
        
        // 闂佺懓鐏氶…鍥鸿箛娑樻嵍闁靛鍎辩拋鏌ユ偡濞嗗繑顥℃い顐ｎ殕閹棃鏁冮埀顒勫船?
        if (projX > -sizeX && projX < w + sizeX) {
            visionCtx.fillStyle = "rgba(202, 138, 4, 0.85)";
            visionCtx.strokeStyle = "#a16207";
            visionCtx.lineWidth = 2;
            
            visionCtx.beginPath();
            visionCtx.rect(projX - sizeX/2, projY - sizeY, sizeX, sizeY);
            visionCtx.fill();
            visionCtx.stroke();
            
            // 缂傚倷鐒﹂敋闁糕晜顨婂畷锝嗗緞鎼淬垻鐩冮梺鍦帛鐢帡鎮?
            visionCtx.fillStyle = "#080a10";
            const holeW = sizeX / 4;
            const holeH = sizeY / 2.5;
            visionCtx.fillRect(projX - sizeX/2.8, projY - sizeY/1.8, holeW, holeH);
            visionCtx.fillRect(projX + sizeX/2.8 - holeW, projY - sizeY/1.8, holeW, holeH);
            
            // 3. 闂佸憡鐟ф慨瀛樻叏?YOLO 闁荤姴娲ゅΛ妤呭春閸℃娴?(Cyan 濠?
            const confidence = Math.min(0.98, 0.99 - (cameraX * 0.02)).toFixed(2);
            
            visionCtx.strokeStyle = "var(--cyan)";
            visionCtx.lineWidth = 2;
            visionCtx.strokeRect(projX - sizeX/2 - 4, projY - sizeY - 4, sizeX + 8, sizeY + 8);
            
            // 闂佸憡鐟﹂崹鐢稿储濠婂牊鈷撻柡澶嬪灥椤?
            visionCtx.shadowColor = "var(--cyan)";
            visionCtx.shadowBlur = 6;
            visionCtx.strokeRect(projX - sizeX/2 - 4, projY - sizeY - 4, sizeX + 8, sizeY + 8);
            visionCtx.shadowBlur = 0;
            
            // 闂佸搫绉村ú銊╊敆閻戣棄妫橀柛銉戝懏鎲?
            visionCtx.fillStyle = "var(--cyan)";
            visionCtx.font = "bold 11px sans-serif";
            const labelText = `[Pallet] ${confidence} d:${cameraX.toFixed(1)}m`;
            visionCtx.fillText(labelText, projX - sizeX/2 - 4, projY - sizeY - 10);
            
            state.vision.detectedObjects.push({
                label: "闂佺懓鐏氶…鍥?(Pallet)",
                conf: confidence,
                dist: cameraX.toFixed(2) + " m",
                pos: `X:${(state.pose.x+dx).toFixed(1)}, Y:${(state.pose.y+dy).toFixed(1)}`
            });
        }
    }
    
    // 4. 濠碘槅鍨崜婵堚偓姘懇瀹曠兘濡搁妸褏顔撴繝銏ｆ硾鐎氼剝銇愰弻銉﹀殑闁稿﹦鍠撳Σ鏇㈡煕閹烘繂浜滈柛鈺佹湰缁?(Person) 闂傚倸鍟扮划顖滀焊?(闁哄鏅滅粙鏍€侀幋锔界劶闁秆勵殕椤斿洦绻濋弴鐔糕拻闁?
    if (state.isNavigating && Math.random() > 0.3) {
        const px = w * 0.7;
        const py = horizon * 1.05;
        const psW = 35;
        const psH = 75;
        
        visionCtx.fillStyle = "rgba(220, 38, 38, 0.4)";
        visionCtx.strokeStyle = "var(--orange)";
        visionCtx.lineWidth = 1.5;
        visionCtx.beginPath();
        visionCtx.ellipse(px, py, psW/2, psH/2, 0, 0, 2*Math.PI);
        visionCtx.fill();
        visionCtx.stroke();
        
        visionCtx.strokeStyle = "var(--orange)";
        visionCtx.strokeRect(px - psW/2 - 2, py - psH/2 - 2, psW + 4, psH + 4);
        
        visionCtx.fillStyle = "var(--orange)";
        visionCtx.font = "bold 10px sans-serif";
        visionCtx.fillText("[Person] 0.88 d:3.2m", px - psW/2 - 2, py - psH/2 - 6);
        
        state.vision.detectedObjects.push({
            label: "闁荤偞绋戞總鏂啃?(Person)",
            conf: "0.88",
            dist: "3.20 m",
            pos: "X:2.4, Y:-0.8"
        });
    }

    updateYoloUI();
}

// 闂佸憡甯￠弨閬嶅蓟?YOLO 濠碘槅鍋€閸嬫挻绻涢弶鎴剰闁割煈浜為幃?DOM
function updateYoloUI() {
    if (state.vision.detectedObjects.length === 0) {
        dom.visionObjectList.innerHTML = '<div class="label-item text-muted">暂无检测目标</div>';
        return;
    }
    
    let html = "";
    state.vision.detectedObjects.forEach(obj => {
        const label = obj.label || obj.class_name || `class ${obj.class_id ?? "-"}`;
        const conf = Number(obj.conf ?? obj.score ?? obj.confidence ?? 0);
        const dist = obj.dist || (obj.depth_m ? `${Number(obj.depth_m).toFixed(2)} m` : "-");
        const pos = obj.pos || "";
        let tagClass = "box";
        const lowerLabel = String(label).toLowerCase();
        if (lowerLabel.includes("pallet") || label.includes("托盘")) tagClass = "pallet";
        if (lowerLabel.includes("person") || label.includes("人员")) tagClass = "person";
        
        html += `
            <div class="label-item">
                <span class="label-tag ${tagClass}">${label}</span>
                <span class="label-conf text-green">${(conf * 100).toFixed(0)}%</span>
                <span class="label-dist font-mono">距离 ${dist} ${pos ? `(${pos})` : ""}</span>
            </div>
        `;
    });
    dom.visionObjectList.innerHTML = html;
}

// ==========================================================================
// 9. VLM 闂佺厧顨庢禍鐐哄礉瑜忛幏鐘活敇閳锯偓閺嬪懎霉濠у灝鈧挾鑺?(Thinking 闂佽鍓氱换鍕庨鈧弻褔宕ｉ妷褏鎲块梺鍦焾濞诧綁骞庨妶鍡欘浄閻犲洦褰冮～銈夋煕閹烘垶顥＄悮?
// ==========================================================================
function setChatMode(mode) {
    const selected = mode === "vlm" ? "vlm" : "model";
    state.vlm.chatMode = selected;
    if (dom.chatModeModel) dom.chatModeModel.classList.toggle("active", selected === "model");
    if (dom.chatModeVlm) dom.chatModeVlm.classList.toggle("active", selected === "vlm");
    if (dom.chatInput) {
        dom.chatInput.placeholder = selected === "model"
            ? "输入叉车执行指令，例如：搬运托盘到卸货点"
            : "输入视觉问答，例如：画面里有什么";
    }
    if (dom.vlmReasoningContent) {
        dom.vlmReasoningContent.innerText = selected === "model"
            ? "大模型用于解析叉车任务，并同步驱动仿真与实机调试。"
            : "视觉大模型通过 ROSBridge 发送 /prompt_text 并接收 /tts_text。";
    }
}

function handleVlmChatInput() {
    const text = dom.chatInput.value.trim();
    if (!text) return;

    addUserMessage(text);
    dom.chatInput.value = "";

    if (state.vlm.chatMode === "model") {
        sendModelNaturalLanguage(text);
        return;
    }

    sendVlmNaturalLanguage(text);
}

function addUserMessage(text) {
    const msgDiv = document.createElement("div");
    msgDiv.className = "chat-message user";
    msgDiv.innerHTML = `<div class="message-content">${text}</div>`;
    dom.chatMessages.appendChild(msgDiv);
    dom.chatMessages.scrollTop = dom.chatMessages.scrollHeight;
}

function addBotMessage(text) {
    const msgDiv = document.createElement("div");
    msgDiv.className = "chat-message bot";
    msgDiv.innerHTML = `<div class="message-content">${text}</div>`;
    dom.chatMessages.appendChild(msgDiv);
    dom.chatMessages.scrollTop = dom.chatMessages.scrollHeight;
    return msgDiv;
}

function updateActiveBotMessage(text) {
    let msgDiv = state.vlm.activeBotMessageEl;
    if (!msgDiv || !dom.chatMessages.contains(msgDiv)) {
        msgDiv = addBotMessage(text);
        state.vlm.activeBotMessageEl = msgDiv;
        return;
    }
    const content = msgDiv.querySelector(".message-content");
    if (content) content.innerText = text;
    dom.chatMessages.scrollTop = dom.chatMessages.scrollHeight;
}

function runSimVlmParser(prompt) {
    state.vlm.isThinking = true;
    const text = prompt.toLowerCase();
    let reasoningChain = [];
    let tasksToCreate = [];

    if (["搬", "托盘", "货物", "carry", "pallet", "transport"].some((word) => text.includes(word))) {
        reasoningChain = [
            "1. 识别到搬运意图，目标是托盘/货物转运。",
            "2. 规划流程：导航到取货区 -> 视觉搜索 -> 抬叉取货 -> 导航到卸货点 -> 放下货物。",
            "3. 已生成本地仿真任务，同时页面仍会保留实机调试能力。",
        ];
        tasksToCreate = [
            { type: "NAV", name: "导航到取货区", params: { x: -4.5, y: -1.0, yaw: Math.PI } },
            { type: "SEARCH", name: "YOLO 搜索托盘", params: { target: "pallet" } },
            { type: "LIFT", name: "抬叉取货", params: {} },
            { type: "NAV", name: "导航到卸货点", params: { x: -8.0, y: 5.5, yaw: Math.PI / 2 } },
            { type: "DROP", name: "放下货物", params: {} },
        ];
    } else if (["巡检", "巡视", "扫描", "patrol", "scan"].some((word) => text.includes(word))) {
        reasoningChain = [
            "1. 识别到巡检/扫描意图。",
            "2. 规划四个导航点覆盖当前区域。",
            "3. 已生成巡检任务队列。",
        ];
        tasksToCreate = [
            { type: "NAV", name: "巡检点 1", params: { x: -6.0, y: -2.0, yaw: 0 } },
            { type: "NAV", name: "巡检点 2", params: { x: 5.0, y: -2.0, yaw: Math.PI / 2 } },
            { type: "NAV", name: "巡检点 3", params: { x: 5.0, y: 5.0, yaw: Math.PI } },
            { type: "NAV", name: "巡检点 4", params: { x: -6.0, y: 5.0, yaw: -Math.PI / 2 } },
        ];
    } else if (["充电", "回充", "dock", "charge"].some((word) => text.includes(word))) {
        reasoningChain = [
            "1. 识别到回充意图。",
            "2. 规划导航到充电区。",
        ];
        tasksToCreate = [
            { type: "NAV", name: "导航到充电区", params: { x: -8.0, y: -8.0, yaw: Math.PI } },
        ];
    } else {
        reasoningChain = [
            "1. 当前指令没有形成明确可执行任务。",
            "2. 可以尝试输入：搬运托盘到卸货点、巡检仓库、回充。",
        ];
    }

    let currentStep = 0;
    dom.vlmReasoningContent.innerText = "";

    function showNextStep() {
        if (currentStep < reasoningChain.length) {
            dom.vlmReasoningContent.innerText += (currentStep > 0 ? "\n" : "") + reasoningChain[currentStep];
            currentStep += 1;
            setTimeout(showNextStep, 350);
            return;
        }

        state.vlm.isThinking = false;
        if (tasksToCreate.length > 0) {
            clearTaskQueue();
            tasksToCreate.forEach((task) => addTask(task.type, task.params, task.name));
            addBotMessage(`已生成 ${tasksToCreate.length} 个任务，并开始执行。`);
            startTaskQueue();
        } else {
            addBotMessage("我还没有解析出可执行任务。可以试试：搬运托盘到卸货点。 ");
        }
    }

    setTimeout(showNextStep, 200);
}

function normalizeTaskYaw(yaw = 0) {
    return Math.abs(yaw) > Math.PI * 2 ? yaw * Math.PI / 180 : yaw;
}

function addTask(type, params = {}, customName = "") {
    let name = customName;
    if (!name) {
        switch (type) {
            case "NAV":
                name = `导航到 (${params.x ?? 0}, ${params.y ?? 0})`;
                break;
            case "SEARCH":
                name = `YOLO 搜索 ${params.target || "目标"}`;
                break;
            case "LIFT":
                name = "抬叉取货";
                break;
            case "DROP":
                name = "放下货物";
                break;
            case "PATROL":
                name = "区域巡检";
                break;
            default:
                name = type || "任务";
        }
    }

    const task = { type, name, params, status: "PENDING" };
    state.tasks.push(task);
    renderTaskTimeline();
    addLog("INFO", "TASK", `新增任务: ${name}`);
}

function clearTaskQueue() {
    state.tasks = [];
    state.activeTaskIndex = -1;
    state.isTaskRunning = false;
    cancelNavigation();
    renderTaskTimeline();
    addLog("WARNING", "TASK", "任务队列已清空。 ");
}

function startTaskQueue() {
    if (state.tasks.length === 0) {
        addLog("WARNING", "TASK", "任务队列为空。 ");
        return;
    }
    state.isTaskRunning = true;
    addLog("SUCCESS", "TASK", "任务队列开始执行。 ");
    executeNextTask(state.activeTaskIndex === -1 ? 0 : state.activeTaskIndex);
}

function pauseTaskQueue() {
    state.isTaskRunning = false;
    if (state.activeTaskIndex !== -1 && state.tasks[state.activeTaskIndex]) {
        state.tasks[state.activeTaskIndex].status = "PENDING";
    }
    cancelNavigation();
    renderTaskTimeline();
    addLog("WARNING", "TASK", "任务队列已暂停。 ");
}

function executeNextTask(index) {
    if (!state.isTaskRunning) return;
    if (index >= state.tasks.length) {
        state.isTaskRunning = false;
        state.activeTaskIndex = -1;
        renderTaskTimeline();
        addLog("SUCCESS", "TASK", "全部任务完成。 ");
        addBotMessage("任务队列执行完成。 ");
        return;
    }

    state.activeTaskIndex = index;
    state.tasks.forEach((task, idx) => {
        if (idx < index) task.status = "COMPLETED";
        if (idx === index) task.status = "ACTIVE";
        if (idx > index) task.status = "PENDING";
    });
    renderTaskTimeline();

    const task = state.tasks[index];
    addLog("INFO", "TASK", `执行任务 ${index + 1}: ${task.name}`);
    runSimTaskAction(task, index);
}

function runSimTaskAction(task, index) {
    switch (task.type) {
        case "NAV": {
            const yaw = normalizeTaskYaw(task.params.yaw || 0);
            sendNavigationGoal(task.params.x || 0, task.params.y || 0, yaw);
            break;
        }
        case "PATROL": {
            const points = task.params.waypoints || [];
            const generated = points.map(([x, y], idx) => ({
                type: "NAV",
                name: `巡检点 ${idx + 1}`,
                params: { x, y, yaw: 0 },
                status: "PENDING",
            }));
            state.tasks.splice(index, 1, ...generated);
            executeNextTask(index);
            break;
        }
        case "SEARCH":
            state.vision.cameraPan = -30.0;
            addLog("INFO", "VISION", "YOLO 正在搜索目标。 ");
            setTimeout(() => {
                if (!state.isTaskRunning || state.activeTaskIndex !== index) return;
                state.vision.cameraPan = 0.0;
                addLog("SUCCESS", "VISION", "已完成目标搜索。 ");
                taskCompleted(index);
            }, 1200);
            break;
        case "LIFT":
            dom.forkliftHeightRange.value = 100;
            state.forklift.targetHeight = 100;
            addLog("INFO", "ACTUATOR", "货叉上升。 ");
            setTimeout(() => {
                if (!state.isTaskRunning || state.activeTaskIndex !== index) return;
                state.forklift.payload = 15;
                state.forklift.statusText = "已载货 (15.0 kg)";
                if (simEnvironment.simPallet) simEnvironment.simPallet.loaded = true;
                taskCompleted(index);
            }, 1200);
            break;
        case "DROP":
            dom.forkliftHeightRange.value = 0;
            state.forklift.targetHeight = 0;
            addLog("INFO", "ACTUATOR", "货叉下降。 ");
            setTimeout(() => {
                if (!state.isTaskRunning || state.activeTaskIndex !== index) return;
                state.forklift.payload = 0;
                state.forklift.statusText = "空载 (0 kg)";
                if (simEnvironment.simPallet) {
                    simEnvironment.simPallet.loaded = false;
                    simEnvironment.simPallet.x = state.pose.x + Math.cos(state.pose.yaw) * 0.8;
                    simEnvironment.simPallet.y = state.pose.y + Math.sin(state.pose.yaw) * 0.8;
                }
                taskCompleted(index);
            }, 1200);
            break;
        default:
            taskCompleted(index);
    }
}

function taskCompleted(index) {
    if (state.activeTaskIndex === index && state.isTaskRunning && state.tasks[index]) {
        state.tasks[index].status = "COMPLETED";
        addLog("SUCCESS", "TASK", `任务完成: ${state.tasks[index].name}`);
        executeNextTask(index + 1);
    }
}

function renderTaskTimeline() {
    if (state.tasks.length === 0) {
        dom.tasksTimeline.innerHTML = '<div class="timeline-empty">暂无任务，可手动添加或通过大模型生成。</div>';
        return;
    }

    const badgeMap = {
        ACTIVE: "执行中",
        COMPLETED: "已完成",
        FAILED: "失败",
        PENDING: "等待",
    };
    let html = "";
    state.tasks.forEach((task, idx) => {
        const statusClass = `task-${String(task.status || "PENDING").toLowerCase()}`;
        const badgeText = badgeMap[task.status] || "等待";
        html += `
            <div class="timeline-item-card ${statusClass}">
                <div>
                    <span class="task-name">${task.name}</span>
                    <span class="task-detail">${task.type}</span>
                </div>
                <div class="task-status-row">
                    <span class="task-badge">${badgeText}</span>
                    ${task.status !== "COMPLETED" ? `<button class="btn-task-del" onclick="deleteTask(${idx})">删</button>` : ""}
                </div>
            </div>
        `;
    });
    dom.tasksTimeline.innerHTML = html;
}

window.deleteTask = function(index) {
    if (state.activeTaskIndex === index && state.isTaskRunning) {
        addLog("WARNING", "TASK", "当前任务正在执行，先暂停后再删除。 ");
        return;
    }
    state.tasks.splice(index, 1);
    if (state.activeTaskIndex > index) state.activeTaskIndex -= 1;
    renderTaskTimeline();
};

// ========================================================================== 
// 11. Main loops
// ========================================================================== 
function updateLoop() {
    if (state.isSimMode) {
        if (state.isNavigating) {
            const dx = state.targetPose.x - state.pose.x;
            const dy = state.targetPose.y - state.pose.y;
            const dist = Math.hypot(dx, dy);

            if (dist < 0.08) {
                state.isNavigating = false;
                state.speed.vx = 0;
                state.speed.vy = 0;
                state.speed.wz = 0;
                addLog("SUCCESS", "NAVIGATION", "到达导航目标。 ");

                if (state.isTaskRunning && state.activeTaskIndex !== -1) {
                    const activeTask = state.tasks[state.activeTaskIndex];
                    if (activeTask && activeTask.type === "NAV") {
                        taskCompleted(state.activeTaskIndex);
                    }
                }
            } else {
                const angleToGoal = Math.atan2(dy, dx);
                const diffAngle = angleToGoal - state.pose.yaw;
                const speed = Math.min(state.speedLimit, dist * 0.5);
                state.speed.vx = speed * Math.cos(diffAngle);
                state.speed.vy = speed * Math.sin(diffAngle);

                let yawDiff = state.targetPose.yaw - state.pose.yaw;
                yawDiff = Math.atan2(Math.sin(yawDiff), Math.cos(yawDiff));
                state.speed.wz = Math.max(-state.yawLimit, Math.min(state.yawLimit, yawDiff * 1.5));
            }
        }

        const dt = (CONFIG.loopRateMs / 1000) * state.simStepScale;
        const deltaWorldX = (state.speed.vx * Math.cos(state.pose.yaw) - state.speed.vy * Math.sin(state.pose.yaw)) * dt;
        const deltaWorldY = (state.speed.vx * Math.sin(state.pose.yaw) + state.speed.vy * Math.cos(state.pose.yaw)) * dt;

        state.pose.x += deltaWorldX;
        state.pose.y += deltaWorldY;
        state.pose.yaw += state.speed.wz * dt;
        state.pose.yaw = Math.atan2(Math.sin(state.pose.yaw), Math.cos(state.pose.yaw));
    }

    const hDiff = state.forklift.targetHeight - state.forklift.height;
    if (Math.abs(hDiff) > 1) {
        state.forklift.height += Math.sign(hDiff) * 2;
        dom.forkliftHeightRange.value = state.forklift.height;
    }

    drawMap();
    updateVisionCanvas();
}

function updateOdomLoop() {
    dom.valVx.innerText = state.speed.vx.toFixed(2) + " m/s";
    dom.valVy.innerText = state.speed.vy.toFixed(2) + " m/s";
    dom.valWz.innerText = state.speed.wz.toFixed(2) + " rad/s";

    dom.robotX.innerText = state.pose.x.toFixed(2) + " m";
    dom.robotY.innerText = state.pose.y.toFixed(2) + " m";
    dom.robotYaw.innerText = (state.pose.yaw * 180 / Math.PI).toFixed(1) + " deg";

    const heightInMm = (state.forklift.height * 3.5).toFixed(0);
    dom.forkliftHeightVal.innerText = heightInMm + " mm";
    dom.forkliftPayloadVal.innerText = state.forklift.statusText;
    dom.forkliftPayloadVal.className = state.forklift.payload > 0 ? "value text-orange" : "value text-green";
}

function updateTelemetryLoop() {
    if (state.isSimMode) {
        state.rdkTelemetry.cpu = Math.max(10, Math.min(95, state.rdkTelemetry.cpu + Math.floor((Math.random() - 0.5) * 6)));
        state.rdkTelemetry.ram = Math.max(20, Math.min(80, state.rdkTelemetry.ram + Math.floor((Math.random() - 0.5) * 2)));
        state.rdkTelemetry.temp = Math.max(38, Math.min(75, state.rdkTelemetry.temp + Math.floor((Math.random() - 0.5) * 4)));

        state.battery.percent = Math.max(2, state.battery.percent - (state.isTaskRunning || state.isNavigating ? 0.05 : 0.01));
        const distToDock = Math.hypot(state.pose.x + 8.0, state.pose.y + 8.0);
        if (distToDock < 0.5) {
            state.battery.percent = Math.min(100, state.battery.percent + 2.0);
            addLog("INFO", "POWER", "正在充电。 ");
        }

        state.battery.voltage = (state.battery.percent / 100 * 3.4 + 13.0).toFixed(1);
        state.stm32Telemetry.uartOk = true;
        const baseRPM = (Math.abs(state.speed.vx) + Math.abs(state.speed.vy) + Math.abs(state.speed.wz)) * 200;
        state.stm32Telemetry.motors = [0, 1, 2, 3].map(() => Math.max(0, Math.floor(baseRPM + (Math.random() - 0.5) * 10)));
    }

    dom.rdkCpu.innerText = state.rdkTelemetry.cpu + "%";
    dom.rdkCpuBar.style.width = state.rdkTelemetry.cpu + "%";
    dom.rdkRam.innerText = state.rdkTelemetry.ram + "%";
    dom.rdkRamBar.style.width = state.rdkTelemetry.ram + "%";
    dom.rdkTemp.innerText = state.rdkTelemetry.temp + " C";
    dom.rdkTempBar.style.width = state.rdkTelemetry.temp + "%";

    if (state.rdkTelemetry.temp > 65) {
        dom.rdkTempBar.className = "progress-fill bg-red";
        dom.rdkTemp.className = "value text-red";
    } else {
        dom.rdkTempBar.className = "progress-fill bg-orange";
        dom.rdkTemp.className = "value text-orange";
    }

    dom.stmYaw.innerText = (state.pose.yaw * 180 / Math.PI).toFixed(1) + " deg";
    dom.stmOdom.innerText = `X: ${state.pose.x.toFixed(2)}m, Y: ${state.pose.y.toFixed(2)}m`;

    const motors = state.stm32Telemetry.motors;
    dom.stmMotors.innerText = `${motors[0]}/${motors[1]}/${motors[2]}/${motors[3]} RPM`;
    dom.stmUartStatus.innerText = state.stm32Telemetry.uartOk ? "在线 (115200 bps)" : "离线";
    dom.stmUartStatus.className = state.stm32Telemetry.uartOk ? "value text-green" : "value text-red";

    const batPct = Math.floor(state.battery.percent);
    dom.batteryPercent.innerText = batPct + "%";
    dom.batteryBar.style.width = batPct + "%";
    dom.batteryVoltage.innerText = `${state.battery.voltage} V (${(Number(state.battery.voltage) / 4).toFixed(2)}V/cell)`;

    if (batPct < 15) {
        dom.batteryPercent.className = "value-large text-red";
        dom.batteryBar.className = "progress-fill bg-red";
        dom.batterySvg.className = "text-red pulse-red";
        dom.alertBanner.classList.remove("hidden");
        dom.alertMessage.innerText = `电量过低 (${batPct}%)，请尽快回充。`;
    } else if (batPct < 30) {
        dom.batteryPercent.className = "value-large text-orange";
        dom.batteryBar.className = "progress-fill bg-orange";
        dom.batterySvg.className = "text-orange";
        dom.alertBanner.classList.add("hidden");
    } else {
        dom.batteryPercent.className = "value-large text-green";
        dom.batteryBar.className = "progress-fill bg-green";
        dom.batterySvg.className = "text-green";
        dom.alertBanner.classList.add("hidden");
    }
}

// ========================================================================== 
// 12. Logs
// ========================================================================== 
function addLog(level, tag, msg) {
    const timeStr = new Date().toTimeString().split(' ')[0];
    const row = document.createElement("div");
    row.className = `log-row ${level.toLowerCase()}`;
    
    row.innerHTML = `
        <span class="log-time">${timeStr}</span>
        <span class="log-tag">[${tag.toUpperCase()}]</span>
        <span class="log-msg">${msg}</span>
    `;
    
    dom.logContainer.appendChild(row);
    // 闂傚倸瀚崝鏇㈠春濡ゅ懎瀚夐柍褜鍓氬鍕槼婵☆偀鏅炵粻娑樜旈崟鍨杸闂?100 闁荤偞绋戦惌澶屾濠靛洨顩烽柕澶嗘櫆浜涘┑鐘欎礁鐏╅柛銈庡幖閻ｇ兘鍩￠崒婊呮
    if (dom.logContainer.childNodes.length > 100) {
        dom.logContainer.removeChild(dom.logContainer.firstChild);
    }
    dom.logContainer.scrollTop = dom.logContainer.scrollHeight;
}
