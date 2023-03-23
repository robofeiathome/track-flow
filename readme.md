# Yolov8_Tracking

## Descrição

Este programa utiliza a engine da YOLOv8 e oferece alguns programas de tracking, sendo definido por padrão como StrongSORT. Para a detecção, o programa utiliza o peso da YOLOv8 para segmentação por padrão, mas também é possível utilizar pesos de detecção normais ou pesos de segmentação personalizados, alterando o caminho e o nome do peso. Este método apresentou uma perda de ID menor em comparação com o DeepSORT, porém ainda não é perfeito e, às vezes, ocorre perda de ID.

## Instalação

__Caso deseje usar uma GPU para rodar o programa, certifique-se de que seu CUDA esteja instalado e configurado corretamente.__ Para verificar se seu CUDA está instalado e qual a sua versão, rode o comando abaixo no terminal:

$ nvcc --version

Caso não funcione, instale o NVIDIA Toolkit. Tutorial de instalação disponível no [site da Nvidia](https://developer.nvidia.com/cuda-downloads).

### track

Para facilitar o setup e uso do programa, um arquivo "requirements.txt" foi adicionado. Primeiro, vá até o repositório e entre na pasta do Yolov8_Tracking.

$ cd hera_vision/
$ cd Yolov8_Tracking/

Em seguida, instale os requerimentos.

$ pip install -r requirements.txt

Pronto, os pacotes para rodar o track estão prontos, mas ainda é necessário instalar as bibliotecas para que o YOLO rode.

### Yolo

Ainda na pasta "Yolov8_Tracking", vá para a pasta do yolov8.

$ cd yolov8/

E por fim, instale tanto o Ultralytics quanto os requerimentos.

$ pip install ultralytics
$ pip install -r requirements.txt

Agora, todos os pacotes para rodar o programa estão instalados.

## Uso 

Você pode rodar o programa de tracking de duas maneiras: pelo arquivo Python usando as configurações setadas nele, ou informando os parâmetros na linha de comando.

### Arquivo .py

No terminal, vá até a pasta Yolov8_Tracking e rode o programa.

$ python3 track.py

Em alguns segundos, o programa começará a rodar. Ele irá identificar o dispositivo CUDA (caso exista) e informará tanto o nome quanto a memória dedicada disponível. Caso não encontre um dispositivo CUDA, ele informará que está rodando com a CPU.

### Alterando os parametros

No terminal, vá até a pasta Yolov8_Tracking. Dentro da pasta, basta rodar como no exemplo anterior do arquivo .py, mas adicionando na frente do comando o parâmetro a ser definido e o valor a ser dado para ele. Por exemplo:

$ python3 track.py --yolo-weights yolov8n.pt # bboxes 
                        yolov8n-seg.pt  # bboxes + segmentacao

Todos os parâmetros que podem ser alterados estão listados dentro do código entre as linhas 315 e 351. Sempre que for adicionar mais um parâmetro e um valor, seguir o mesmo padrão, dando espaço entre o valor anterior e o próximo parâmetro. Exemplo:

$ python3 track.py --source 0 --yolo-weights yolov8n.pt --img 640

Também será possível ver os parâmetros aqui no arquivo README.md, que serão listados em seguida:

## Parametros 

__Segue a lista dos argumentos e suas definições padrão. Alguns recebem True ou False, enquanto outros recebem valores numéricos (exceto pelo CUDA, que pode receber "cpu")__

 - yolo-weights (default=WEIGHTS / 'yolov8s-seg.pt')
 - reid-weights (default=WEIGHTS / 'osnet_x0_25_msmt17.pt')
 - tracking-method (default='bytetrack')
 - tracking-config (type=Path, default=None)
 - source (default='0')
 - imgsz (default=[640])
 - conf-thres (default=0.5)
 - iou-thres (default=0.5)
 - max-det (default=1000)
 - device (default='')
 - show-vid (action='store_true')
 - save-txt (action='store_true')
 - save-conf (action='store_true')
 - save-crop (action='store_true')
 - save-trajectories (action='store_true')
 - save-vid (action='store_true')
 - nosave (action='store_true')
 - classes
 - agnostic-nms (action='store_true')
 - augment (action='store_true')
 - visualize (action='store_true')
 - update (action='store_true')
 - project (default=ROOT / 'runs' / 'track')
 - name (default='exp')
 - exist-ok (action='store_true')
 - line-thickness (default=2)
 - hide-labels (default=False, action='store_true')
 - hide-conf (default=False, action='store_true')
 - hide-class (default=False, action='store_true')
 - half (action='store_true')
 - dnn (action='store_true')
 - vid-stride (type=int, default=1)
 - retina-masks (action='store_true')
