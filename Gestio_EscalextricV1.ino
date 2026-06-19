/* =============================================================
   Programa TFG: Node micro-ROS per al control de 4 pistes d'Scalextric
   =============================================================
   Placa:      ESP32-C3 DevKit
   Connexió:   Wi-Fi (es connecta a l'Access Point de la Raspberry Pi)

   Funció:
     - Rep comandes de velocitat per pista des de ROS 2 (topic "vel_pista_escalextric").
     - Tradueix la velocitat lògica (0-1000) a un senyal PWM cap a cada pista.
     - Publica periòdicament l'estat actual (topic "publica_prueba").

   Flux de dades:
     Raspberry Pi (Agent micro-ROS)  <-- Wi-Fi -->  ESP32-C3 (aquest node)
   ============================================================= */

#include <micro_ros_arduino.h>            // Pont entre Arduino i micro-ROS
#include <stdio.h>
#include <rcl/rcl.h>                       // Capa base de ROS Client Library
#include <rclc/rclc.h>                     // Utilitats d'alt nivell (suport, nodes...)
#include <rclc/executor.h>                 // Executor: gestiona els callbacks
#include <std_msgs/msg/int32_multi_array.h> // Tipus de missatge: array d'enters

// =============================================================
//   CONFIGURACIÓ DE XARXA WI-FI I AGENT ROS 2
// =============================================================
// Ens connectem a la xarxa generada per l'adaptador USB de la Raspberry
const char SSID[] = "WIFI_SCALEXTRIC";
const char WIFI_PASS[] = "12345678";

// IP de la Raspberry Pi (Gateway del Punt d'Accés) i Port de l'Agent
IPAddress agent_ip(192, 168, 4, 1);
const size_t agent_port = 8888; 

// =============================================================
//   CONFIGURACIÓ DE MAQUINARI (PINS ESP32-C3)
// =============================================================
// Pins de sortida PWM, un per pista (GPIO de l'ESP32-C3)
#define PWM_PISTA_1 0
#define PWM_PISTA_2 1
#define PWM_PISTA_3 3
#define PWM_PISTA_4 4
#define LED_PIN 8 // LED Integrat de l'ESP32-C3 (indicador d'estat)

// Macros de seguretat micro-ROS
// RCCHECK:     si la crida falla, salta al bucle d'error (parada total)
// RCSOFTCHECK: si la crida falla, l'ignora i continua (errors no crítics)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// =============================================================
//   VARIABLES DEL SISTEMA
// =============================================================
int pista = 1;        // Pista seleccionada per l'última comanda rebuda (1-4)
int velocidad = 0;    // Velocitat de l'última comanda rebuda (0-1000)
int velocidad_1 = 0;  // Velocitat actual memoritzada de la pista 1
int velocidad_2 = 0;  // Velocitat actual memoritzada de la pista 2
int velocidad_3 = 0;  // Velocitat actual memoritzada de la pista 3
int velocidad_4 = 0;  // Velocitat actual memoritzada de la pista 4

// Objectes micro-ROS
rcl_subscription_t sub_comandos;              // Subscriptor: rep les comandes de velocitat
std_msgs__msg__Int32MultiArray msg_comandos;  // Buffer del missatge entrant

rcl_publisher_t pub_estado;                   // Publicador: envia l'estat actual
std_msgs__msg__Int32MultiArray msg_estado;    // Buffer del missatge sortint

rclc_executor_t executor;   // Executor que despatxa els callbacks dels subscriptors
rclc_support_t support;     // Context i recursos de suport de rclc
rcl_allocator_t allocator;  // Gestor de memòria de ROS
rcl_node_t node;            // El node ROS 2 que representa aquest ESP32

// Variables per al control de temps (No bloquejant)
unsigned long tempsUltimaPublicacio = 0;
const unsigned long INTERVAL_PUBLICACIO = 1000; 
bool estatLed = false;

// =============================================================
//   FUNCIONS
// =============================================================

// Bucle d'error (Parpelleig ràpid si no es pot connectar a ROS)
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100); 
  }
}

// Callback de recepció: s'executa cada cop que arriba una nova comanda de la Raspberry.
// El missatge esperat és [pista, velocitat].
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;

   if (msg->data.size >= 2) {      // Només processem si el missatge té els 2 camps
    pista = msg->data.data[0];     // Primer camp: número de pista (1-4)
    velocidad = msg->data.data[1]; // Segon camp: velocitat desitjada (0-1000)
    
    // Evitem valors bojos per seguretat (Límit 0 a 1000)
    velocidad = constrain(velocidad, 0, 1000); 
    
    // Traduïm la velocitat lògica a resolució real de 10 bits de l'ESP32 (0-1023)
    uint32_t pwm_duty = map(velocidad, 0, 1000, 0, 1023);
      
    if (pista == 1) {
        velocidad_1 = velocidad;
        ledcWrite(PWM_PISTA_1, pwm_duty);
    } else if (pista == 2) {
        velocidad_2 = velocidad;
        ledcWrite(PWM_PISTA_2, pwm_duty);
    } else if (pista == 3) {
        velocidad_3 = velocidad;
        ledcWrite(PWM_PISTA_3, pwm_duty);
    } else if (pista == 4) {
        velocidad_4 = velocidad;
        ledcWrite(PWM_PISTA_4, pwm_duty);
    }
  }
}

// =============================================================
//   SETUP
// =============================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED encès fix mentre intenta connectar al Wi-Fi i ROS

  // --- Configuració PWM (Core 3.x) ---
  // Freqüència de 20.000 Hz i resolució de 10 bits per a cada pista
  ledcAttach(PWM_PISTA_1, 20000, 10);
  ledcAttach(PWM_PISTA_2, 20000, 10);
  ledcAttach(PWM_PISTA_3, 20000, 10);
  ledcAttach(PWM_PISTA_4, 20000, 10);
  
  // Apagar els motors a l'inici per seguretat
  ledcWrite(PWM_PISTA_1, 0);
  ledcWrite(PWM_PISTA_2, 0);
  ledcWrite(PWM_PISTA_3, 0);
  ledcWrite(PWM_PISTA_4, 0);

  // --- CONNEXIÓ WI-FI AL MICRO-ROS AGENT ---
  set_microros_wifi_transports((char*)SSID, (char*)WIFI_PASS, agent_ip, agent_port);

  delay(2000); // Temps de marge perquè la xarxa s'estabilitzi

  // --- Inicialització de l'estructura ROS 2 ---
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "ctrl_escalextric", "", &support));

  // Crear el Publicador (Per enviar l'estat)
  RCCHECK(rclc_publisher_init_default(&pub_estado,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),"publica_prueba"));
  msg_estado.data.capacity = 2;
  msg_estado.data.size = 2;
  msg_estado.data.data = (int32_t *)malloc(sizeof(int32_t) * 2);

  // Crear el Subscriptor (Per rebre la velocitat)
  RCCHECK(rclc_subscription_init_default(&sub_comandos,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),"vel_pista_escalextric"));
  msg_comandos.data.capacity = 2;
  msg_comandos.data.size = 2;
  msg_comandos.data.data = (int32_t *)malloc(sizeof(int32_t) * 2);
   
  // Crear l'Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_comandos, &msg_comandos, &subscription_callback, ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW); // Apaga el LED quan tot està llest i connectat!
}

// =============================================================
//   LOOP (Temps Real)
// =============================================================
void loop() {
  
  // 1. ESCOLTAR ROS (Instantani)
  // Revisa la bústia durant 10ms. Si arriba una ordre, crida a "subscription_callback"
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  // 2. TEMPORITZADOR NO BLOQUEJANT (Publicar dades de fons)
  unsigned long tempsActual = millis();

  // Només entra aquí dins 1 cop cada segon
  if (tempsActual - tempsUltimaPublicacio >= INTERVAL_PUBLICACIO) {
    tempsUltimaPublicacio = tempsActual;

    // Parpelleig del LED de validació d'activitat (Opcional, es pot comentar si molesta)
    estatLed = !estatLed;
    digitalWrite(LED_PIN, estatLed ? HIGH : LOW);
    
    // Prepara i envia l'estat actual al servidor
    msg_estado.data.data[0] = pista; 
    msg_estado.data.data[1] = velocidad; 
    RCSOFTCHECK(rcl_publish(&pub_estado, &msg_estado, NULL));
  }
}