// BIBLIOTECAS PADRÃO DO C++ 

#include <algorithm>   // funções utilitárias genéricas — aqui usada pelo std::max() em joint_state_callback(), pra pegar a maior velocidade entre os dois dedos de um mesmo par

#include <chrono>      // biblioteca de tempo do C++. Permite escrever "20ms" e "0s"

#include <cmath>       // funções matemáticas em C — usada pelo std::abs() comouble (diferença de ângulo, velocidade da junta)

#include <memory>      // ponteiros inteligentes (std::shared_ptr, std::make_shared).

#include <string>      // tipo std::string — usado nas variáveis modelo_garra_, link_garra_ etc, os nomes que identificam robô/objeto pro Link Attacher

#include <unordered_map>  // estrutura chave->valor (tipo dicionário). Usada em joint_state_callback() pra montar um mapa nome_da_junta -> velocidade, e buscar rápido pelo nome


// BIBLIOTECAS DO ROS2

#include "rclcpp/rclcpp.hpp" // Biblioteca cliente do ROS2 pra C++ ("ROS Client Library"). 
                             // Dá acesso a rclcpp::Node, create_publisher, create_subscription, create_wall_timer, CLCPP_INFO/WARN etc.

#include "sensor_msgs/msg/joint_state.hpp" // Define o tipo de mensagem JointState: nome, posição, velocidade e esforço de cada junta do robô. É o que joint_state_sub_ recebe do tópico joint_states (publicado pelo joint_state_broadcaster) pra saber se uma junta travou.

#include "std_msgs/msg/float64_multi_array.hpp" // Define Float64MultiArray, um array simples de números decimais. É o formato que o fin_ray_controller (JointGroupPositionController) espera receber em /fin_ray_controller/commands — por isso publisher_ publica um array de 4 posições (uma por junta revolute).

#include "std_msgs/msg/string.hpp" // Define String para o tipo do tópico /comando_garra, por onde chegam os comandos "abrir" e "fechar".

#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"
// Vêm do pacote linkattacher_msgs (parte do IFRA Link Attacher). 
// Define os serviços AttachLink e DetachLink: a estrutura de request/response usada pra pedir ao plugin do Gazebo pra "colar" ou "descolar" a garra no objeto. 

using namespace std::chrono_literals; // Habilita escrever "20ms" e "0s" direto (sufixos literais de tempo), em vez de std::chrono::milliseconds(20) por extenso.


// CLASSE PRINCIPAL DO NÓ 

class GripperMover : public rclcpp::Node
// Herda de rclcpp::Node: toda a "infraestrutura" de nó ROS2 (nome, logger, timers, publishers, subscriptions) vem de graça por essa herança.
{
public:
  GripperMover()
  : Node("gripper_mover")  // nome do nó como aparece em "ros2 node list"
  {
    // ---------- Comunicação ROS2 ----------

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/fin_ray_controller/commands", 10);
    // Publica no tópico que o controller (position_controllers/JointGroupPositionController) escuta.
    // O "10" é o tamanho da fila de mensagens (QoS depth).

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/comando_garra", 10,
      std::bind(&GripperMover::comando_callback, this, std::placeholders::_1));
    // Assina o tópico /comando_garra: toda mensagem "abrir"/"fechar" que chegar aqui dispara comando_callback().

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GripperMover::joint_state_callback, this, std::placeholders::_1));
    // Assina o estado real das juntas (publicado pelo joint_state_broadcaster), usado pra detectar quando um dedo travou contra o objeto.

    attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    // Clientes de serviço: não escutam nada continuamente, só chamam o serviço quando chamar_attach()/chamar_detach() são acionados.

    // Identificação do modelo/objeto no Gazebo
    // Esses nomes precisam bater exatamente com os nomes usados no .world e no spawn_entity do launch file, senão o Link Attacher não encontra os links pra colar.

    modelo_garra_ = "fin_ray_gripper";
    modelo_objeto_ = "objeto_teste";
    link_objeto_ = "link";
    objeto_colado_ = false;   // estado: o objeto está colado na garra agora?
    tentando_colar_ = false;  // trava pra não disparar attach() duas vezes em paralelo

    link_garra_ = "base_garra";
    // Link da garra ao qual o objeto será fixado: a base, não um dedo específico, pra não depender de qual par travou primeiro.

    // ---------- Ângulos de referência (radianos) ----------
    // Os 4 dedos se movem em pares espelhados: 1 e 4 giram num sentido, 2 e 3 no sentido oposto, pra fechar simetricamente em torno do centro.

    angulo_aberto_2_3_ = 0.5;
    angulo_fechado_2_3_ = -0.325;

    angulo_aberto_1_4_ = -0.5;
    angulo_fechado_1_4_ = 0.325;

    // ---------- Estado inicial de movimento ----------
    // "atual" = posição que está sendo publicada agora (parte do meio de aproxima())
    // "alvo"  = pra onde o dedo está indo no momento

    atual_1_4_ = angulo_aberto_1_4_;
    atual_2_3_ = angulo_aberto_2_3_;
    alvo_1_4_ = angulo_aberto_1_4_;
    alvo_2_3_ = angulo_aberto_2_3_;

    // ---------- Estado de detecção de travamento ----------

    inicio_fechamento_1_4_ = atual_1_4_;  // de onde começou o fechamento atual
    inicio_fechamento_2_3_ = atual_2_3_;
    travado_1_4_ = false;   // esse par já travou (encostou em algo) neste fechamento?
    travado_2_3_ = false;
    fechando_ = false;      // está em processo de fechar agora?

    contagem_baixa_1_4_ = 0;   // quantos ticks seguidos de velocidade baixa (candidatos a trava)
    contagem_baixa_2_3_ = 0;
    ticks_desde_fechar_ = 0;   // contador de segurança (timeout do fechamento)

    velocidade_junta_1_4_ = 0.0;  // última velocidade angular lida do /joint_states
    velocidade_junta_2_3_ = 0.0;

    // ---------- Parâmetros de movimento e detecção ----------

    velocidade_ = 0.8; // Velocidade angular alvo de fechamento/abertura, em rad/s (aprox).

    limiar_velocidade_travamento_ = 0.02; // Abaixo dessa velocidade angular, a junta é candidata a "travada" (achou resistência = provável contato com objeto).

    distancia_minima_antes_de_checar_ = 0.05; // Evita falso-positivo: só passa a checar travamento depois que o dedo 
                                              // já percorreu essa distância angular desde o início do fechamento

    timer_period_ = 20ms; // Período do loop de controle: a cada 20ms, timer_callback() roda de novo (50Hz).

    passo_ = velocidade_ * (timer_period_.count() / 1000.0);
    // Quanto ângulo avançar a cada tick, dado a velocidade desejada e o período do timer. 
    // timer_period_.count() vem em ms, por isso o /1000.0 pra converter pra segundos antes de multiplicar pela velocidade (rad/s).

    timer_ = this->create_wall_timer(
      timer_period_, std::bind(&GripperMover::timer_callback, this));
    // Cria o timer que dispara timer_callback() repetidamente do nó, roda o loop de controle em tempo real.

    RCLCPP_INFO(this->get_logger(),
      "gripper_mover pronto. Publique 'abrir' ou 'fechar' em /comando_garra"); // Log informativo, aparece no terminal quando o nó sobe com sucesso.
  }

private:

  // Callback disparado toda vez que chega uma mensagem em /comando_garra.
  void comando_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "abrir") {
      // Manda os dois pares de volta ao ângulo aberto e reseta as travas, já que abrir "esquece" qualquer contato anterior.
      alvo_1_4_ = angulo_aberto_1_4_;
      alvo_2_3_ = angulo_aberto_2_3_;
      travado_1_4_ = false;
      travado_2_3_ = false;
      fechando_ = false;
      if (objeto_colado_) {
        chamar_detach();  // se tinha objeto colado, solta antes de abrir
      }
    } else if (msg->data == "fechar") {
      alvo_1_4_ = angulo_fechado_1_4_;
      alvo_2_3_ = angulo_fechado_2_3_;
      inicio_fechamento_1_4_ = atual_1_4_;  // marca de onde o fechamento começou
      inicio_fechamento_2_3_ = atual_2_3_;  // (usado por checa_travamento)
      travado_1_4_ = false;
      travado_2_3_ = false;
      fechando_ = true;
      contagem_baixa_1_4_ = 0;   // zera os contadores pra um fechamento novo
      contagem_baixa_2_3_ = 0;
      ticks_desde_fechar_ = 0;
    } else {
      // Qualquer comando que não seja "abrir" nem "fechar" é ignorado, só avisa no log e sai sem alterar nenhum estado.
      RCLCPP_WARN(this->get_logger(),
        "Comando desconhecido: '%s' (use 'abrir' ou 'fechar')", msg->data.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Comando '%s' recebido, movendo aos poucos...",
      msg->data.c_str());
  }

  // Callback disparado a cada nova leitura de /joint_states, atualiza a velocidade conhecida das juntas revolute, usada pra detectar trava.
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::unordered_map<std::string, double> velocidades;
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
      velocidades[msg->name[i]] = msg->velocity[i];
      // A mensagem JointState traz duas listas paralelas (name[] e velocity[]) pra todas as juntas do robô (incluindo as prismáticas).
    }

    // Lambda auxiliar: busca a velocidade de uma junta pelo nome; se não achar (por exemplo, ainda não chegou nenhuma leitura), retorna 0.0.
    auto pega = [&](const std::string & nome) {
      auto it = velocidades.find(nome);
      return it != velocidades.end() ? std::abs(it->second) : 0.0;
    };

    // Guarda a MAIOR velocidade entre os dois dedos do par, se qualquer um dos dois ainda estiver se movendo rápido, o par inteiro ainda não é considerado travado.
    velocidade_junta_1_4_ = std::max(
      pega("dedo1_base_to_joint_1_revolute"),
      pega("dedo4_base_to_joint_1_revolute"));

    velocidade_junta_2_3_ = std::max(
      pega("dedo2_base_to_joint_1_revolute"),
      pega("dedo3_base_to_joint_1_revolute"));
  }

  // Move "atual" um passo_ em direção a "alvo", sem ultrapassar: suavidade do movimento.
  double aproxima(double atual, double alvo)
  {
    double diferenca = alvo - atual;
    if (std::abs(diferenca) <= passo_) {
      return alvo;  // já está a menos de um passo do alvo: encosta nele direto
    }
    return atual + (diferenca > 0 ? passo_ : -passo_);
    // Avança passo_ na direção certa (positiva ou negativa conforme o sinal da diferença).
  }

  // Detecta se um par de dedos travou (achou resistência) durante o fechamento. 
  // Recebe/altera por referência o estado daquele par específico.
  bool checa_travamento(
    bool fechando, double atual, double inicio, double velocidade,
    bool & travado, double & alvo, int & contagem_baixa)
  {
    if (!fechando || travado) {
      return false;  // só faz sentido checar durante o fechamento e se ainda não travou
    }
    double ja_andou = std::abs(atual - inicio);
    bool distancia_ok = ja_andou > distancia_minima_antes_de_checar_;// só considera a checagem válida depois de um mínimo de percurso
    bool velocidade_baixa = velocidade < limiar_velocidade_travamento_;// velocidade real da junta está abaixo do limiar = sinal de resistência

    if (distancia_ok && velocidade_baixa) {
      ++contagem_baixa;
      // soma mais uma leitura consecutiva de velocidade baixa
    } else {
      contagem_baixa = 0;
      // qualquer leitura "normal" no meio zera o contador, evita confirmar trava por causa de um pico isolado de ruído
    }

    if (contagem_baixa >= LEITURAS_CONSECUTIVAS_) {
      // várias leituras seguidas de velocidade baixa = trava confirmada
      travado = true;
      alvo = atual;  // trava o alvo na posição atual, pra parar de tentar fechar mais
      RCLCPP_INFO(this->get_logger(),
        "Trava detectada (objeto?) -- parando de fechar em %.3f rad", atual);
      return true;
    }
    return false;
  }

  // Loop principal de controle, chamado pelo timer_ a cada 20ms.
  void timer_callback()
  {
    // Verifica travamento em cada par de dedos independentemente
    checa_travamento(fechando_, atual_1_4_, inicio_fechamento_1_4_,
      velocidade_junta_1_4_, travado_1_4_, alvo_1_4_, contagem_baixa_1_4_);
    checa_travamento(fechando_, atual_2_3_, inicio_fechamento_2_3_,
      velocidade_junta_2_3_, travado_2_3_, alvo_2_3_, contagem_baixa_2_3_);

    // Se está fechando há tempo demais sem travar (por exemplo, não tem objeto nenhum no caminho)
    // força a parada na posição atual em vez de ficar fechando indefinidamente.
    if (fechando_) {
      ++ticks_desde_fechar_;
      if (ticks_desde_fechar_ >= MAX_TICKS_FECHAMENTO_) {
        if (!travado_1_4_) {
          travado_1_4_ = true;
          alvo_1_4_ = atual_1_4_;
        }
        if (!travado_2_3_) {
          travado_2_3_ = true;
          alvo_2_3_ = atual_2_3_;
        }
      }
    }

    // Só tenta colar o objeto quando OS DOIS pares já travaram (garra realmente fechou em torno de algo dos dois lados),
    // e evita chamar de novo se já está colado ou já tem uma chamada em andamento.
    if (travado_1_4_ && travado_2_3_ && !objeto_colado_ && !tentando_colar_) {
      chamar_attach();
    }

    // Avança a posição publicada um passo em direção ao alvo de cada par
    atual_1_4_ = aproxima(atual_1_4_, alvo_1_4_);
    atual_2_3_ = aproxima(atual_2_3_, alvo_2_3_);

    // Publica o comando de posição pras 4 juntas revolute, na ordem esperada pelo controller.yaml: dedo1, dedo2, dedo3, dedo4.
    // Cada par se move espelhado (mesmo valor, dedos diferentes).
    auto out = std_msgs::msg::Float64MultiArray();
    out.data = {atual_1_4_, atual_2_3_, atual_2_3_, atual_1_4_};
    publisher_->publish(out);
  }

  // Pede ao plugin Link Attacher pra criar uma junta fixa temporária entre a garra e o objeto — só é chamado quando os dois pares já travaram.
  void chamar_attach()
  {
    if (!attach_client_->wait_for_service(0s)) {
      // wait_for_service(0s) = checagem instantânea, não bloqueia esperando.
      // Se o serviço não existe, o plugin provavelmente não está carregado no .world file.
      RCLCPP_WARN(this->get_logger(),
        "/ATTACHLINK indisponível -- o plugin do LinkAttacher está no .world?");
      return;
    }

    tentando_colar_ = true;
    // marca que já tem uma chamada em andamento, pra timer_callback() não disparar chamar_attach() de novo enquanto essa ainda não respondeu

    auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    req->model1_name = modelo_garra_;
    req->link1_name = link_garra_;
    req->model2_name = modelo_objeto_;
    req->link2_name = link_objeto_;
    // Monta o pedido: "conecte este link deste modelo com aquele link daquele outro modelo" 
    // é o que o plugin usa pra saber quais dois corpos unir fisicamente no Gazebo.

    attach_client_->async_send_request(req,
      [this](rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedFuture future) {
        // Chamada assíncrona: não trava o nó esperando resposta. Esse lambda roda quando a resposta chegar (sucesso ou falha).
        auto resp = future.get();
        tentando_colar_ = false;
        objeto_colado_ = resp->success;
        if (resp->success) {
          RCLCPP_INFO(this->get_logger(), "Colado: %s", resp->message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Falha ao colar: %s", resp->message.c_str());
        }
      });
  }

  // Pede ao plugin Link Attacher pra desfazer a junta fixa
  // Chamado quando o comando "abrir" chega e o objeto ainda está colado.
  void chamar_detach()
  {
    if (!detach_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(),
        "/DETACHLINK indisponível -- o plugin do LinkAttacher está no .world?");
      return;
    }

    objeto_colado_ = false; // Marca como descolado já de imediato 

    auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    req->model1_name = modelo_garra_;
    req->link1_name = link_garra_;
    req->model2_name = modelo_objeto_;
    req->link2_name = link_objeto_;

    detach_client_->async_send_request(req,
      [this](rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedFuture future) {
        auto resp = future.get();
        RCLCPP_INFO(this->get_logger(), "Descolado: %s", resp->message.c_str());
      });
  }

  // VARIÁVIES DE ESTADO DE NÓ

  // --- Comunicação ROS2 ---
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;

  // --- Identificação no Gazebo (Link Attacher) ---
  std::string modelo_garra_;
  std::string link_garra_;
  std::string modelo_objeto_;
  std::string link_objeto_;
  bool objeto_colado_;     // true = objeto atualmente fixado na garra
  bool tentando_colar_;    // true = chamada de attach em andamento (evita duplicar)

  // --- Ângulos de referência (abertos/fechados) por par de dedos ---
  double angulo_aberto_2_3_;
  double angulo_fechado_2_3_;
  double angulo_aberto_1_4_;
  double angulo_fechado_1_4_;

  // --- Posição atual publicada e posição-alvo, por par ---
  double atual_1_4_;
  double atual_2_3_;
  double alvo_1_4_;
  double alvo_2_3_;

  // --- Estado da detecção de travamento, por par ---
  double inicio_fechamento_1_4_;
  double inicio_fechamento_2_3_;
  bool travado_1_4_;
  bool travado_2_3_;
  bool fechando_;              // true enquanto um comando "fechar" está em curso

  int contagem_baixa_1_4_;     // leituras consecutivas de velocidade baixa (par 1&4)
  int contagem_baixa_2_3_;     // idem, par 2&3
  int ticks_desde_fechar_;     // contador de segurança/timeout do fechamento

  static constexpr int LEITURAS_CONSECUTIVAS_ = 5; // quantas leituras seguidas de velocidade baixa confirmam a trava

  static constexpr int MAX_TICKS_FECHAMENTO_ = 150; // 150 ticks * 20ms = 3s: tempo máximo de fechamento antes de desistir e travar na posição atual mesmo sem detectar contato

  // --- Velocidades lidas de /joint_states e parâmetros de detecção ---
  double velocidade_junta_1_4_;
  double velocidade_junta_2_3_;
  double limiar_velocidade_travamento_;
  double distancia_minima_antes_de_checar_;

  // --- Parâmetros de movimento suave ---
  double velocidade_;               // velocidade angular alvo (rad/s aprox.)
  double passo_;                    // incremento angular por tick do timer
  std::chrono::milliseconds timer_period_;  // período do loop de controle
};

// FUNÇÃO PRINCIPAL

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Inicializa o contexto do ROS2 (parseia argumentos de linha de comando reservados do ROS, configura comunicação DDS)

  rclcpp::spin(std::make_shared<GripperMover>());
  // Cria o nó e entra no loop de eventos: mantém o processo vivo, processando callbacks (timer, subscriptions, respostas de serviço) até ser interrompido (Ctrl+C)

  rclcpp::shutdown();
  // Encerra o contexto do ROS2 de forma limpa antes de sair
  return 0;
}
