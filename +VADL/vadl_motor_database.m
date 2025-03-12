function [L_motor, D_motor, m_motor0, m_prop0, was_motor_defined] = vadl_motor_database(motortype)

% motor geometry
was_motor_defined = false;
if motortype == "H340"
    L_motor= 0.365; % motor length [m] L1720
    D_motor= 0.029; % motor diameter [m] L1720
    m_motor0 = 0.391; % total motor initial mass [kg] 1720
    m_prop0 = 0.207; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "H550"
    L_motor= 0.206; % motor length [m] K1100
    D_motor= 0.038; % motor diameter [m] K1100
    m_motor0= 0.316; % total motor initial mass [kg] K1100
    m_prop0= 0.176; % initial motor propellent mass [kg] K1100
    was_motor_defined = true;
end

if motortype == "I245"
    L_motor= 0.202; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.365; % total motor initial mass [kg] 1720
    m_prop0 = 0.181; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "I287"
    L_motor= 0.302; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.605; % total motor initial mass [kg] 1720
    m_prop0 = 0.331; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "I345"
    L_motor= 0.203; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.343; % total motor initial mass [kg] 1720
    m_prop0 = 0.174; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "I357"
    L_motor= 0.203; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.343; % total motor initial mass [kg] 1720
    m_prop0 = 0.174; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "I366"
    L_motor= 0.298; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.556; % total motor initial mass [kg] 1720
    m_prop0 = 0.300; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "I430"
    L_motor= 0.292; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.540; % total motor initial mass [kg] 1720
    m_prop0 = 0.240; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "I435"
    L_motor= 0.299; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.518; % total motor initial mass [kg] 1720
    m_prop0 = 0.270; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "K1100"
    L_motor= 0.398; % motor length [m] K1100
    D_motor= 0.054; % motor diameter [m] K1100
    m_motor0= 1.336; % total motor initial mass [kg] K1100
    m_prop0= .773; % initial motor propellent mass [kg] K1100
    was_motor_defined = true;
end

if motortype == "K1800"
    L_motor= 0.492; % motor length [m] K1100
    D_motor= 0.075; % motor diameter [m] K1100
    m_motor0= 2.772; % total motor initial mass [kg] K1100
    m_prop0= 1.113; % initial motor propellent mass [kg] K1100
    was_motor_defined = true;
end

if motortype == "K1800_VDF_2"
    L_motor= 0.492; % motor length [m] K1100
    D_motor= 0.075; % motor diameter [m] K1100
    m_motor0= 2.772; % total motor initial mass [kg] K1100
    m_prop0= 1.113; % initial motor propellent mass [kg] K1100
    was_motor_defined = true;
end

if motortype == "L1400"
    L_motor= 0.726; % motor length [m] L1720
    D_motor= 0.054; % motor diameter [m] L1720
    m_motor0 = 2.540; % total motor initial mass [kg] 1720
    m_prop0 = 1.400; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "L1520"
    L_motor= 0.518; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 3.651; % total motor initial mass [kg] 1720
    m_prop0 = 1.854; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "L1720"
    L_motor= 0.486; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 3.341; % total motor initial mass [kg] 1720
    m_prop0 = 1.755; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "L1720_VDF_Thrust"
    L_motor= 0.486; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 3.341; % total motor initial mass [kg] 1720
    m_prop0 = 1.755; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "L1720_USLI"
    L_motor= 0.486; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 3.341; % total motor initial mass [kg] 1720
    m_prop0 = 1.755; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end


if motortype == "L2200"
    L_motor= 0.681; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 4.783; % total motor initial mass [kg] 1720
    m_prop0 = 2.518; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L2050"
    L_motor= 1.114; % motor length [m] L1720
    D_motor= 0.054; % motor diameter [m] L1720
    m_motor0 = 4.056; % total motor initial mass [kg] 1720
    m_prop0 = 2.338; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L1940"
    L_motor= 0.560; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    %m_motor0 = 3.211 % total motor initial mass [kg] 1720 % THIS DOES NOT MATCH THRUST CURVE BUT IT IS CORRECT - WE CONTACTED THE MANUFACTURER 10/24/22
    m_motor0 = 3.856; % total motor initial mass [kg]
    m_prop0 = 1.825; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L2350"
    L_motor= 0.210; % motor length [m] L1720
    D_motor= 0.152; % motor diameter [m] L1720
    m_motor0 = 5.550; % total motor initial mass [kg] 1720
    m_prop0 = 1.680; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L1355"
    L_motor= 0.621; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 4.962; % total motor initial mass [kg] 1720
    m_prop0 = 3.012; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L1170"
    L_motor= 0.665; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 4.990; % total motor initial mass [kg] 1720
    m_prop0 = 2.800; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L1350"
    L_motor= 0.486; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 3.571; % total motor initial mass [kg] 1720
    m_prop0 = 1.905; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L1420"
    L_motor= 0.665; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 4.562; % total motor initial mass [kg] 1720
    m_prop0 = 2.560; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L1685"
    L_motor= 0.757; % motor length [m] L1685
    D_motor= 0.075; % motor diameter [m] L1685
    m_motor0 = 6.051; % total motor initial mass [kg] L1685
    m_prop0 = 3.773; % initial motor propellent mass [kg] L1685
    was_motor_defined = true;
end
if motortype == "L2500"
    L_motor= 0.443; % motor length [m] L1720
    D_motor= 0.098; % motor diameter [m] L1720
    m_motor0 = 4.989; % total motor initial mass [kg] 1720
    m_prop0 = 2.313; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L2375"
    L_motor= 0.621; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 4.161; % total motor initial mass [kg] 1720
    m_prop0 = 2.322; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L2525"
    L_motor= 1.524; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 5.579; % total motor initial mass [kg] 1720
    m_prop0 = 1.306; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "L3150"
    L_motor= 0.394; % motor length [m] L1720
    D_motor= 0.098; % motor diameter [m] L1720
    m_motor0 = 4.731; % total motor initial mass [kg] 1720
    m_prop0 = 2.386; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "O8000"
    L_motor= 0.957; % motor length [m] L1720
    D_motor= 0.161; % motor diameter [m] L1720
    m_motor0 = 32.672; % total motor initial mass [kg] 1720
    m_prop0 = 18.610; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "J396"
    L_motor= 0.406; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.752; % total motor initial mass [kg] 1720
    m_prop0 = 0.372; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "J335"
    L_motor= 0.367; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.627; % total motor initial mass [kg] 1720
    m_prop0 = 0.342; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "I566"
    L_motor= 0.245; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.384; % total motor initial mass [kg] 1720
    m_prop0 = 0.198; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "I540"
    L_motor= 0.367; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.598; % total motor initial mass [kg] 1720
    m_prop0 = 0.309; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end
if motortype == "I470"
    L_motor= 0.303; % motor length [m] L1720
    D_motor= 0.038; % motor diameter [m] L1720
    m_motor0 = 0.495; % total motor initial mass [kg] 1720
    m_prop0 = 0.247; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "K2045"
    L_motor= 0.404; % motor length [m] L1720
    D_motor= 0.054; % motor diameter [m] L1720
    m_motor0 = 1.290; % total motor initial mass [kg] 1720
    m_prop0 = 0.716; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end

if motortype == "K2000"
    L_motor= 0.350; % motor length [m] L1720
    D_motor= 0.075; % motor diameter [m] L1720
    m_motor0 = 2.465; % total motor initial mass [kg] 1720
    m_prop0 = 1.102; % initial motor propellent mass [kg] 1720
    was_motor_defined = true;
end


if was_motor_defined == false
    fprintf("Motor dimensions not provided! Please find and code in lengths and masses.\n");
end
