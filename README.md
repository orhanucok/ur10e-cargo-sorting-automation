# Gerçekçi UR10e Koli Ayırma Hücresi

Bu proje, üç gerçek Universal Robots UR10e kolunun bir koli yönlendirme ve ayırma
hücresinde sanal devreye alınması için hazırlanmıştır.

## Çalışan simülasyon

- CoppeliaSim 4.10 Pro ve MuJoCo, 5 ms fizik adımı
- Universal Robots'un resmî UR10e görsel, çarpışma, kütle ve atalet verileri
- 3× UR10e, prosedürel Robotiq EPick dört kaplı vakum tutucu
- 1,05 m çelik silindir kaideler, üst/alt flanşlar ve ankraj cıvataları
- 1× Basler ace 2 ana kamera ve 3× RealSense D455 yerel kamera
- Üstte ileri, uçlarda aşağı, altta geri dönen kapalı çevrimli kinematik bant paletleri
- Giriş sepetini sürekli besleyen fiziksel galvaniz oluk, yan korkuluklu üst magazin ve
  MuJoCo yerçekimiyle gerçekten kayıp düşen koli havuzu
- Farklı kütleli fiziksel koliler; koliyle birlikte dönen, ince ve çarpışmasız etiket/barkod geometrisi
- Giriş UR10e'sinde gerçek koli seçimi, vakum kavrama, sabit kontrol istasyonuna bırakma,
  D455 yön kontrolü, gerektiğinde etiketi üste çevirme ve hareketli banda fiziksel aktarım
- Giriş sepetinde 18° titreşimli yerçekimi tabanı ve UR10e erişim sınırı denetimi; koliler
  rastgele uzak köşelerde kalarak robotu kilitlemez
- Kırmızı ve mavi görev rezervasyonu, bant takibi, motor hedefli IK hareketi, vakum alma ve
  sepet içine fiziksel bırakma
- Giriş ve ayırma noktalarında fiziksel korkuluk açıklıkları ve robot–konveyör çarpışmasında
  bandı durdurup robotu dikey geri çeken 20 Hz otomatik emniyet kurtarması
- Gerçek TCP hedefe ulaşmadan durum değiştirmeyen hareket denetimi, 40 saniyelik robot
  watchdog'u ve kontrolcü kalp atışını izleyen bağımsız sağlık gözcüsü
- Tek tıklamayla temiz sahne kuran masaüstü kısayolu

Masaüstündeki `UR10e Koli Ayirma Simulasyonu` kısayoluna çift tıklamak yeterlidir.
Başlatıcı testleri çalıştırır, resmî robot dosyasını doğrular, CoppeliaSim'i bulur ve tam
hücreyi yeniden kurup başlatır. Başlatıcı penceresi başarıdan sonra kendiliğinden kapanır;
CoppeliaSim açık kalır.

## Kesintisiz çalışma ve otomatik toparlanma

Kısayol, simülasyondan ayrı çalışan gizli bir sağlık gözcüsü de başlatır. Gözcü iki saniyede
bir kontrolcü kalp atışını, üretim ilerlemesini ve CoppeliaSim durumunu denetler. Beklenmeyen
pause otomatik devam ettirilir; kontrolcü kalp atışı 20 saniye ilerlemezse veya 60 simülasyon
saniyesi boyunca üretim olmazsa sahne temiz biçimde yeniden kurulur. Kısayolla yapılan bilinçli
kurulumlarda bakım kilidi kullanıldığı için gözcüyle sahne kurucu aynı anda çalışmaz.

Gözcü olayları `outputs/simulation_watchdog.log` dosyasına kaydedilir. Normal çalışmada
`sorting_cell_safety_collision`, `sorting_cell_progress_stalled` ve robot watchdog sayaçları
`0` kalır.

Eğimli besleyici sonsuz sayıda sahne nesnesi üretmez. Aynı fiziksel koli magazinden giriş
sepetine, kontrol istasyonuna, hareketli banda, ana kameraya ve renk sepetine kadar ilerler;
çıktı sepetindeki bekleme süresinden sonra havuza dönerek yeniden kullanılır. Böylece uzun süre
açık kalan simülasyonda RAM kullanımı sürekli büyümez. Bırakma aralığı, havuz büyüklüğü ve
çıktı sepetinde kalma süresi `config/cell.yaml` içindeki `infeed_supply` bölümünden değiştirilebilir.

## Komut satırından çalıştırma

```powershell
.\scripts\launch_project.ps1
```

Çekirdek testleri tek başına çalıştırmak için:

```powershell
.\work\.venv\Scripts\python.exe -m pytest -q
```

## Doğruluk sınırı

Şu anki sahne gerçek MuJoCo temas fiziği, gerçek UR10e geometrisi/dinamiği ve çalışan
görev akışı içerir. Vakum kavrama, temas mesafesi doğrulandıktan sonra sert bir tutma
bağıyla modellenir. Ana kameranın renk kararı ve giriş D455'in etiket-yönü kontrolü bu aşamada
koli pozundan alınan simülasyon ground-truth bilgisini kullanır; gerçek piksel tabanlı
OpenCV/OCR düğümü ve ROS 2 Jazzy +
MoveIt 2 entegrasyonu sonraki devreye alma katmanıdır. Bu ayrım, görüntü işleme başarı
oranının fizik demonstrasyonuyla karıştırılmaması için kasıtlıdır.

Üretim sonucunu temsil etmek için karton–bant sürtünmesi, vakum sızdırmazlığı, kamera
gecikmesi ve gerçek robot hız/ivme sınırları sahada ölçülerek `config/physics.yaml`
değerleri kalibre edilmelidir.
