# UR10e + CoppeliaSim ile Gerçekçi Koli Yönlendirme ve Ayırma Sistemi

Tarih: 11 Ağustos 2026

## 1. Kesin teknoloji ve donanım seçimi

### Simülasyon programı

- CoppeliaSim V4.10.0 rev0, Linux x86-64
- Ana fizik motoru: MuJoCo
- Hareket planlama: MoveIt 2
- Robot ve sistem haberleşmesi: ROS 2 Jazzy
- Robot kontrolü: ros2_control ve Universal Robots ROS 2 Driver ile aynı eklem/trajectory arayüzleri
- Görüntü işleme: OpenCV; gerekli olursa ONNX Runtime veya YOLO tabanlı etiket algılama
- Ana geliştirme dilleri: C++ (zaman kritik kontrol ve takip), Python (görüntü işleme, test ve koordinasyon)
- İşletim sistemi: Ubuntu 24.04, tercihen Windows yanında native dual boot

### Robotlar

- 3 adet Universal Robots UR10e
- 6 eksen
- 1300 mm erişim
- Model varyantına göre 10 kg veya 12,5 kg azami yük
- Yaklaşık 1 m/s azami takım hızı
- Üretici tekrar edilebilirlik değeri: ±0,05 mm
- Takım: Robotiq EPick, dört vantuzlu konfigürasyon

Üç istasyonda aynı robot kullanmak yedek parça, kinematik, kontrol yazılımı, MoveIt yapılandırması ve gerçek sisteme geçiş açısından avantajlıdır. Tasarımın başlangıç kutu sınırı, takım dahil güvenli çalışma payı bırakmak için 8 kg olarak kabul edilir. Daha ağır veya çok geçirgen karton kullanılırsa UR20/PowerPick değerlendirmesi tekrar yapılmalıdır.

### Kameralar

- 1 adet ana bant kamerası: Basler ace 2 a2A2448-105g5cBAS sınıfı, 5 MP renkli global shutter
- 3 adet robot yerel kamerası: RealSense D455 RGB-D modeli
- Her kameranın gerçek iç parametreleri, lens bozulması, pozlama, kare hızı ve gecikmesi ayrı yapılandırma dosyasında tutulur.
- Ana kamera sınıflandırma ve koli kimliği üretir. Robot kameraları yalnız son poz düzeltmesi, tutuş doğrulaması ve çarpışma güvenliği için kullanılır.

## 2. Bilgisayar uygunluğu

Mevcut bilgisayar: Acer Nitro 5, NVIDIA RTX 4050 Laptop GPU, 16 GB RAM.

Bu sistem CoppeliaSim + MuJoCo için kullanılabilir. Üç robot ve dört kamera aynı anda çalıştırılırken aşağıdaki sınırlar uygulanacaktır:

- Geliştirme görüntüleri: 640×480 veya 960×540
- Ana kamera: 20–30 FPS
- Robot kameraları: 10–15 FPS
- Görsel mesh ile fiziksel collision mesh birbirinden ayrılacak.
- Fizik adımı geliştirmede 5 ms, doğrulama çalıştırmalarında 2 ms olacak.
- Toplu testler görüntü penceresi kapatılarak headless çalıştırılacak.
- Gereksiz yüksek çözünürlüklü dokular kullanılmayacak.

32 GB RAM yükseltmesi zorunlu değildir, fakat ROS 2, CoppeliaSim, dört kamera, derinlik görüntüleri ve geliştirme araçlarını birlikte rahat kullanmak için kuvvetle önerilir. RTX 4050'nin VRAM miktarı `nvidia-smi` ile doğrulanmalıdır.

NVIDIA Isaac Sim seçilmemiştir. Güncel minimum gereksinimi 32 GB RAM, RTX 4080 ve 16 GB VRAM seviyesindedir; mevcut bilgisayarda üç robotlu sahne ve çoklu kamera için uygun değildir.

## 3. “Gerçek dünya simülasyonu” tanımı

Bu projede nesneler koordinatları doğrudan değiştirilerek hareket ettirilmeyecektir. Aşağıdakiler fiziksel olarak modellenir:

- Robot bağlantılarının gerçek kütleleri ve atalet matrisleri
- Eklem hız, ivme, tork ve konum sınırları
- Motor/servo davranışı ve kontrol gecikmesi
- Koli kütlesi, ağırlık merkezi ve atalet momenti
- Karton–bant, karton–platform ve karton–vantuz temasları
- Statik/dinamik sürtünme, geri sekme ve temas yumuşaklığı
- Vakum kuvveti, conta uyumu, sızıntı, kayma ve kopma eşikleri
- Konveyör ivmelenmesi, hızı, titreşimi ve koli kayması
- Kamera çözünürlüğü, görüş açısı, lens bozulması, gürültü, motion blur ve gecikme
- ROS mesaj gecikmesi ve kare kaçırma senaryoları

Gerçek donanım alınmadan kurulan sistem “yüksek doğruluklu fizik tabanlı sanal devreye alma”dır. Tam dijital ikiz sayılması için daha sonra gerçek makineden ölçülen sürtünme, vakum, motor ve kamera sonuçlarıyla kalibre edilmesi gerekir.

## 4. Hat yerleşimi

Başlangıç sahne boyutu yaklaşık 7,0 × 3,5 metredir.

1. Giriş koli sepeti
2. UR10e-01 besleme/yönlendirme robotu
3. Hareketsiz yönlendirme platformu
4. Hareketli konveyör başlangıcı
5. Tek ana Basler kamera
6. UR10e-02 kırmızı ayırma bölgesi
7. Kırmızı sepet
8. UR10e-03 mavi ayırma bölgesi
9. Mavi sepet
10. Okunamayan veya kaçırılan kutular için reject alanı

Konveyör başlangıç değerleri:

- Uzunluk: 5 m
- Bant genişliği: 0,7 m
- Çalışma yüksekliği: 0,8 m
- Başlangıç hızı: 0,15 m/s
- Sürekli çalışma hedefi: 0,20–0,30 m/s
- Minimum koli aralığı: ilk doğrulamada 450 mm; nihai testte çevrim hesabına göre azaltılır.

MuJoCo yüzey hızı kullanan sahte bir bant yerine, programla döngü halinde ilerletilen kinematik bant elemanları/slatlar kullanacaktır. Böylece koli hareketi temas ve sürtünmeden doğar.

## 5. Koli modeli

İlk tasarım zarfı:

- En küçük: 200 × 150 × 120 mm
- En büyük: 400 × 300 × 250 mm
- Kütle: 0,5–8 kg
- Etiket herhangi bir yüzde bulunabilir.
- Etiket sınıfları: kırmızı, mavi ve bilinmeyen/reject

Her test başında boyut, kütle, ağırlık merkezi, sürtünme, etiket pozisyonu, ışık ve başlangıç yönelimi belirli sınırlar içinde rastgeleleştirilir. Bu, sistemin tek bir ezberlenmiş sahnede başarılı görünmesini engeller.

## 6. Yönlendirme istasyonu

UR10e-01 sepetten en güvenli üst kutuyu RGB-D nokta bulutu üzerinden seçer. Kutuyu hareketsiz, kaydırmaz yüzeyli bir yönlendirme platformuna koyar.

Platform özellikleri:

- Kutunun yuvarlanırken kaçmasını engelleyen alçak yan dayama
- Değiştirilebilir sürtünme yüzeyi
- Kutunun temas kuvvetlerini kaydeden sanal kuvvet sensörü
- Robotun yan yüzü vakumla kavrayıp 90° devirmesine izin veren açıklık

Etiket yönlendirme algoritması:

1. Üst görüntü ve robot RGB-D görüntüsü alınır.
2. Kutunun 3B oriented bounding box pozu hesaplanır.
3. Görünen etikete ait yüz normali bulunursa etiketi +Z yönüne getiren en kısa dönüş hesaplanır.
4. Etiket görünmüyorsa sistem kontrol ettiği yüzleri hafızada tutarak 90° roll/pitch arama dizisi uygular.
5. Her dönüşten sonra kutu tamamen durana kadar beklenir.
6. Etiketin yukarı baktığı açı ve güven skoru doğrulanır.
7. Robot kutuyu bant başlangıcına koyar ve kutu kimliğini koordinatöre bildirir.

Robot kutuyu ışınlamaz veya görsel olarak döndürmez; vakum tutuşu, kuvvet sınırları ve platform teması üzerinden gerçek hareket üretir.

## 7. Ana kamera ve koli kimliği

Tek ana kamera, iki ayırma robotuna ortak karar verir. Her yeni koli için sanal bir takip kimliği oluşturur:

```text
box_id: BOX_000042
class: BLUE
confidence: 0.997
belt_s: 1.82 m
velocity: 0.151 m/s
assigned_robot: UR10e_03
eta_to_pick: 6.7 s
state: RESERVED
```

Takip sistemi sadece “birinci/ikinci/üçüncü koli” saymaz. Ana kamera görüntüsü, konveyör encoder konumu, Kalman filtresi ve alma bölgesi fotoelektrik sensörü birleştirilir. Böylece kutu kayarsa veya bir görüntü karesi kaçarsa sıra tamamen bozulmaz.

Kamera kırmızı/mavi kararını verdikten sonra:

- Mavi kutu yalnız UR10e-03'e rezerve edilir.
- Kırmızı kutu yalnız UR10e-02'ye rezerve edilir.
- Diğer robot o kimliği ignore listesine alır.
- Güven eşiği altındaki kutu hiçbir renk sepetine gönderilmez.

## 8. Ayırma robotları

Robotlar bandın farklı boyuna konumlarında bulunur; çalışma hacimleri çakıştırılmaz. Her robotta önceden hesaplanmış:

- bekleme pozu,
- izleme pozu,
- hareketli hedefe yaklaşma yörüngesi,
- tutma penceresi,
- güvenli kaldırma pozu,
- sepet bırakma pozu

bulunur.

Dinamik alma sırası:

1. Koordinatör görevi ve tahmini varış zamanını gönderir.
2. Robot bekleme pozundan ön-yaklaşma pozuna geçer.
3. Yerel D455, beklenen kutuyu yakalama bölgesinde doğrular.
4. Encoder ve görüntü kullanılarak kutunun gelecekteki pozu tahmin edilir.
5. Robot TCP'si kısa süre bant hızına senkronlanır.
6. Vakum devreye girer; basınç ve temas doğrulanır.
7. Kutu dikey kaldırılır.
8. Hedef sepete bırakılır.
9. Başarılı sonuç koordinatöre bildirilir.

## 9. Vakum modeli

Basit sabit bağlantı nihai çözüm olmayacaktır. EPick modeli aşağıdaki bileşenleri içerir:

- Vantuz temas alanı
- Vakum basınç farkı
- `F = ΔP × A × seal_quality` normal tutma kuvveti
- Yanal sürtünme sınırı
- Maksimum moment
- Conta yay/sönüm davranışı
- Karton geçirgenliğine bağlı sızıntı
- Pompa yükselme süresi
- Sensör gecikmesi

Tutuş aşağıdaki durumlarda kopar:

- normal yük sınırı aşılırsa,
- yanal kuvvet sürtünme sınırını aşarsa,
- dönme momenti sınırı aşarsa,
- yeterli conta yüzdesi oluşmazsa,
- koli hasarlı/çok geçirgen senaryosunda vakum düşerse.

Bu model sayesinde robot kutuyu yanlış açıyla veya fazla ivmeyle taşırsa simülasyonda gerçekten düşürebilir.

## 10. Yazılım mimarisi

```text
sorting_cell_ws/src/
├── sorting_cell_description
├── sorting_cell_coppelia
├── sorting_cell_interfaces
├── sorting_cell_perception
├── sorting_cell_tracking
├── sorting_cell_coordinator
├── sorting_cell_motion
├── sorting_cell_gripper
├── sorting_cell_bringup
├── sorting_cell_dashboard
└── sorting_cell_tests
```

Temel düğümler:

- `main_camera_node`
- `box_detector_node`
- `box_tracker_node`
- `conveyor_encoder_node`
- `orientation_station_node`
- `task_coordinator_node`
- `ur10e_01_executor`
- `ur10e_02_executor`
- `ur10e_03_executor`
- `vacuum_model_node`
- `safety_supervisor_node`
- `metrics_recorder_node`

Robot işleri ROS 2 action, sürekli sensör verileri topic, kısa sorgular service olarak tasarlanır.

## 11. Durum makineleri

Koli durumları:

```text
IN_BIN
SELECTED
PICKED
AT_ORIENTATION_STATION
ORIENTING
LABEL_UP
ON_CONVEYOR
CLASSIFIED
RESERVED_RED / RESERVED_BLUE
ENTERING_PICK_ZONE
GRASPED
SORTED_RED / SORTED_BLUE
REJECTED
LOST
```

Her koli tek bir aktif sahibine sahip olabilir. Robot başarısız olduğunda görev açıkça iptal/recovery durumuna alınmadan başka robota verilmez.

## 12. Fizik kalibrasyonu

Başlangıçta üretici CAD/URDF değerleri kullanılır. Sonraki kalibrasyonlar:

1. Karton–bant sürtünmesi eğik düzlem testiyle ölçülür.
2. Bant ivmesi encoder verisiyle ölçülür.
3. Koli kütlesi ve ağırlık merkezi tartı/denge testiyle ölçülür.
4. EPick vakum yükselme ve kopma eğrisi gerçek sensör kaydıyla ölçülür.
5. UR10e eklem hareket süreleri gerçek robot veya URSim kaydıyla karşılaştırılır.
6. Kamera lens parametreleri kalibrasyon tahtasıyla ölçülür.
7. Simülasyon ve gerçek çevrim süreleri/kayma mesafeleri karşılaştırılır.

Her parametrenin kaynağı `measured`, `manufacturer`, `estimated` veya `randomized` olarak işaretlenir.

## 13. Geliştirme aşamaları ve kapılar

### Faz 0 — Sistem kurulumu ve performans ölçümü (2–3 gün)

- Ubuntu 24.04 dual boot
- NVIDIA sürücüsü
- CoppeliaSim, ROS 2 Jazzy, MoveIt 2
- RTX 4050 VRAM ve sahne FPS ölçümü
- Git/LFS proje yapısı

Kapı: Boş MuJoCo sahnesi ve üç UR10e, 5 ms fizik adımında kararlı çalışır.

### Faz 1 — Üretici robot modelinin hazırlanması (4–6 gün)

- Resmî UR10e URDF/mesh içe aktarımı
- Collision mesh kontrolü
- Kütle/atalet ve joint limit doğrulaması
- EPick ve kamera montajı
- Tek robot dinamik testleri

Kapı: Robot, konum ışınlamadan tork/hız limitleri içinde 100 yörünmeyi tamamlar.

### Faz 2 — Konveyör ve temas fiziği (5–7 gün)

- Kinematik bant elemanları
- Gerçek kütleli koliler
- Sürtünme ve kayma testleri
- Encoder ve fotoelektrik sensörler

Kapı: 100 koli testinde hız/konum sapması belirlenen tolerans içinde kalır; simülasyon patlaması oluşmaz.

### Faz 3 — Vakum ve pick-place (1–2 hafta)

- EPick kuvvet modeli
- Conta/sızıntı/kopma
- Force-torque kayıtları
- Sepet ve platform tutuşları

Kapı: Geçerli tutuşlar taşınır; geçersiz, köşe veya aşırı ivmeli tutuşlar fiziksel olarak başarısız olur.

### Faz 4 — Sepetten algılama (1–2 hafta)

- RGB-D nokta bulutu
- Kutu segmentasyonu ve oriented bounding box
- Görünür üst kutu seçimi
- Çarpışmasız yaklaşma
- Sıkışmış kutu algılama

Kapı: Rastgele fakat ulaşılabilir 100 sahnenin en az %95'inde güvenli kutu seçimi.

### Faz 5 — Etiketi üste çevirme (1–2 hafta)

- Etiket düzlemi ve yüz normali
- En kısa yönelim dönüşü
- Sistematik altı yüz araması
- Regrasp planlama
- Etiket üst açı doğrulaması

Kapı: Rastgele etiket yüzü/yönelimli 500 testte en az %98 başarı ve yanlış doğrulama olmaması.

### Faz 6 — Ana kamera ve çoklu takip (1–2 hafta)

- Basler intrinsics ve global shutter modeli
- Renk/etiket sınıflandırma
- Kimlik üretme
- Encoder destekli takip
- Kayıp kare ve örtüşme senaryoları

Kapı: 1.000 kolilik temiz testte kimlik değişimi sıfır; düşük güvenli kutular reject edilir.

### Faz 7 — İki ayırma robotu (1–2 hafta)

- Görev rezervasyonu
- Kırmızı/mavi ignore listeleri
- Statik bantta alma
- Yerel kamera doğrulaması
- Robot çalışma hacmi kontrolü

Kapı: 500 kutuda yanlış renk sepeti sıfır, robot çarpışması sıfır.

### Faz 8 — Hareketli banttan alma (2 hafta)

- Gelecek poz tahmini
- TCP-bant hız senkronizasyonu
- Gecikme telafisi
- Kaçırılan koli recovery/reject

Kapı: Bant durmadan 1.000 kolide en az %95 toplama; yanlış koli tutma sıfır.

### Faz 9 — Donanım-zorlayıcı ve hata testleri (1–2 hafta)

- 2 ms fizik adımı
- Kamera gürültüsü ve motion blur
- Değişken sürtünme/kütle
- Vakum kaçağı
- Koli kayması
- Ağ ve işleme gecikmesi
- Bir robotun durması

Kapı: Güvensiz durumda sistem yanlış karar vermek yerine kontrollü durur veya reject uygular.

### Faz 10 — Gerçek robota hazır sanal devreye alma (1 hafta)

- Simülasyon ve gerçek robot arayüzlerinin aynı olması
- UR ROS 2 Driver entegrasyon testi
- Hız/güvenlik limitleri
- Calibration YAML desteği
- Hardware-in-the-loop planı

Kapı: Simülasyon denetleyicisi yalnız yapılandırma değiştirerek URSim veya gerçek UR10e sürücüsüne bağlanabilir.

Toplam gerçekçi geliştirme tahmini: tek geliştirici için 12–16 hafta. Donanım kalibrasyonu ve gerçek makine devreye alma bu sürenin dışındadır.

## 14. Kabul kriterleri

| Ölçüm | Hedef |
|---|---:|
| Etiketi yukarı yönlendirme | ≥ %98 |
| Temiz görüntüde sınıflandırma | ≥ %99,5 |
| Gürültülü görüntüde sınıflandırma | ≥ %98 |
| Koli kimliği değişimi | 1.000 kolide 0 |
| Yanlış renk sepeti | 1.000 kolide 0 |
| Dinamik banttan tutma | ≥ %95 |
| Ulaşılabilir sepette tutma | ≥ %95 |
| Robot-robot çarpışması | 0 |
| Güvensiz vakumla taşıma | 0 |
| Bilinmeyen etiketi renk sepetine atma | 0 |
| Simülasyon deterministik tekrar testi | Tolerans içinde |

## 15. İndirilen ve kullanılacak üretici modeli

Universal Robots'un resmî `Universal_Robots_ROS2_Description` deposunun 4.3.1 etiketi projeye sabitlenmiştir. Paket şunları içerir:

- UR10e görsel DAE ağları
- UR10e collision STL ağları
- fiziksel parametreler
- eklem sınırları
- kinematik parametreler
- görsel parametreler

Bu URDF doğrudan düzenlenmeyecek; proje adaptörleri ve CoppeliaSim'e özel fizik ayarları ayrı paketlerde tutulacaktır. Böylece üretici modeli güncellenebilir ve yapılan kalibrasyonlar kaybolmaz.

## 16. Resmî kaynaklar

- [CoppeliaSim V4.10 ve indirme sayfası](https://www.coppeliarobotics.com/)
- [CoppeliaSim MuJoCo ve diğer fizik motorları](https://manual.coppeliarobotics.com/en/dynamicsModule.htm)
- [CoppeliaSim fizik motoru farkları](https://manual.coppeliarobotics.com/en/physicsEngineDifferences.htm)
- [CoppeliaSim ROS 2 arayüzü](https://manual.coppeliarobotics.com/en/ros2Interface.htm)
- [CoppeliaSim URDF içe aktarma](https://manual.coppeliarobotics.com/en/importExport.htm)
- [Universal Robots resmî ROS 2 description paketi](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [Universal Robots resmî ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [UR10e teknik özellikleri](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/complianceUR10e/H_g5_sections/appendix_g5/tech_spec_sheet.htm)
- [Robotiq EPick ürün verileri](https://robotiq.com/hubfs/Product-sheets/Vacuum%20Grippers/ROBOTIQ_One_Pager_Vacuum_Grippers_10_2024_EN_V3.pdf)
- [RealSense D455 özellikleri](https://www.realsenseai.com/products/real-sense-depth-camera-d455f/)
- [Basler ace 2 a2A2448-105g5cBAS özellikleri](https://docs.baslerweb.com/a2a2448-105g5cbas)
- [Isaac Sim sistem gereksinimleri](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)

## 17. İlk uygulama sırası

1. RAM ve GPU/VRAM doğrulaması
2. Ubuntu 24.04 native kurulum kararı
3. CoppeliaSim lisans uygunluğu
4. CoppeliaSim + MuJoCo performans testi
5. UR10e modelinin tek robot olarak içe aktarılması
6. Gerçek eklem limitleri ve fizik testi
7. EPick kuvvet modeli
8. Konveyör temas modeli
9. Üç robotlu tam sahne
10. Kamera ve koordinasyon yazılımı

Bu sırada ilk hafta sonunda güzel görünen bir video yerine, tek UR10e'nin gerçek limitlerle ve doğrulanmış temas fiziğiyle koli kaldırdığı ölçülebilir bir temel hedeflenir.
