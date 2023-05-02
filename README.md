# Yolov8_Tracking

## Descrição

Este programa utiliza a engine da YOLOv8 e oferece alguns programas de tracking, sendo definido por padrão como StrongSORT. Para a detecção, o programa utiliza o peso da YOLOv8 para segmentação por padrão, mas também é possível utilizar pesos de detecção normais ou pesos de segmentação personalizados, alterando o caminho e o nome do peso. Este método apresentou uma perda de ID menor em comparação com o DeepSORT, porém ainda não é perfeito e, às vezes, ocorre perda de ID.

NOTA: Essa branch, ROS_integration funciona com uma RosCam.

## Instalação

__Caso deseje usar uma GPU para rodar o programa, certifique-se de que seu CUDA esteja instalado e configurado corretamente.__ Para verificar se seu CUDA está instalado e qual a sua versão, rode o comando abaixo no terminal:

```
$ nvcc --version
```
Caso não funcione, instale o NVIDIA Toolkit. Tutorial de instalação disponível no [site da Nvidia](https://developer.nvidia.com/cuda-downloads).

### track

Para facilitar o setup e uso do programa, um arquivo "requirements.txt" foi adicionado. Primeiro, vá até o repositório e entre na pasta do Yolov8_Tracking.

```
$ cd track-flow/
$ cd Yolov8_Tracking/
```
Em seguida, instale os requerimentos.
```
$ pip install -r requirements.txt
```
Agora, todos os pacotes para rodar o programa estão instalados.

# Instalação para usar uma camera em ROS

Nota: Pule isso caso vá utilizar apenas uma webcam.

## Camera Virtual - v4l2loopback

A instalação do módulo que cria a câmera virtual no Linux é bem fácil. Primeiro atualize os pacotes do seu sistema:

```
$ sudo apt-get update
```
Em seguida, instale o módulo

```
$ sudo apt-get install v4l2loopback-dkms
$ sudo apt-get install v4l-utils
```

Para usar a câmera virtual é necessário criar uma. Primeiro verifique os dispositivos que já estão conectados, o próprio pacote oferece um meio de verificar:

```
$ v4l2-ctl --list-devices
```

Recomendo criar uma câmera com um ID diferente das existentes, por mais que possa funcionar, algumas vezes isso vai gerar conflitos que podem ser evitados. Para criar a câmera basta um comando, nele você pode definir os parâmetros da câmera, como o ID, nome, tamanho, quantos dispositivos vão acessar essa câmera, etc. Vou dar um exemplo aqui de como criar uma câmera com o ID 2, assim, nesse caso, quando eu for usar a câmera, basta chamar no source, assim como, uma webcam: '2'

```
$ sudo modprobe v4l2loopback video_nr=2 card_label="RosCam" max_width=640 max_height=480
```
Também adicinei um nome para camera, assim como, a resolução da camera.

## Escrita do tópico na camera

Nota: Pule isso caso vá utilizar apenas uma webcam.

### Verifique o tópico da sua camera

Após inicializar sua câmera em ROS, certifique-se de qual o tópico ela está gerando e qual deles você deseja utilizar, a maioria das câmeras retorna a imagem pura e uma imagem comprimida, aconselho a usar a imagem pura por ser mais fácil de trabalhar. Para listar os tópicos em ROS que estão sendo publicados e verificar o tópico correto da sua câmera, use o comando :
```
$ rostopic list
```
### Intalação

Para transformarmos o tópico da câmera em uma "webcam" é necessário um meio de transformar essa imagem, para isso você pode usar algumas bibliotecas e tentar fazer por conta própria, porém existe um programa funcional que não adiciona nenhum delay considerável (no computador que testei, uma Jetson AGX Xavier, usar esse método adicionou nos picos máximos um delay de 5ms e uma média de 3ms)
Vou colocar aqui um guia de instalação passo a passo, mas caso prefira [aqui está o repo original](https://github.com/jgoppert/ros-virtual-cam).

```
$ cd seu_rep_onde_vai_utilizar_a_camera_em_ros/src
$ mkdir ros/virtual-cam-ws/src -p #pode mudar o nome ou diminuir o caminho, por exemplo, tirando o ros/, funciona eu já testei :)
$ cd ros/virtual-cam-ws/src
$ git clone git@github.com:ixirobot/ros-virtual-cam.git virtual_cam
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

Note: Esse pacote tem um erro no arquivo image_converter.h, para corrigir basta trocar a linha <opencv/cv.hpp> por <opencv2/opencv.hpp>
Agora que tudo está instalado corretamente, certifique-se que você deu launch na sua câmera, verifique se seu tópico está sendo publicado, certifique-se que sua câmera virtual foi criada, e então dê o seguinte comando no terminal para começar a subscrever o tópico na câmera virtual:

```
#Exemplo
$ rosrun virtual_cam stream _device:=/dev/video2 _width:=640 _height:=480 _fourcc:=YV12 image:=/my_camera/image
```
Em _device:= você vai colocar o caminho para a câmera virtual, no caso essa é a com ID 2, mas pode mudar de acordo com sua necessidade, _width e _height são o tamanho da imagem, _fourcc é o formato dos pixels e image é o tópico da sua câmera.

## Uso 

Você pode rodar o programa de tracking de duas maneiras: pelo arquivo Python usando as configurações setadas nele, ou informando os parâmetros na linha de comando.
Você pode rodar programa de tracking de três formas, usando uma webcam (não possui launch, portanto se usa manualmente) usando uma virtual_cam no programa normal, ou usando o super_flow, uma técnica que relaciona a posição na tela e a direção.

### launch

No terminal, vá até o seu rep e dê um source, em seguida inicie o programa (depoisde dar o source você pode dar o launch de qualquer local do terminal)

```
$ roslaunch follow_me follow_vision.launch
$ roslaunch follow_me super_flow.launch

```
Em alguns segundos, o programa começará a rodar. Ele irá identificar o dispositivo CUDA (caso exista) e informará tanto o nome quanto a memória dedicada disponível. Caso não encontre um dispositivo CUDA, ele informará que está rodando com a CPU.

### Parâmetros

Você pode mudar os parâmetros do programa para se adequar à sua necessidade, entre no arquivo "super_flow.launch" para alterar os parâmetros.
