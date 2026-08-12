# Proje Başlangıç Durumu

Tarih: 11 Ağustos 2026

## Tamamlananlar

- Git deposu oluşturuldu.
- RTX 4050 Laptop GPU doğrulandı: 6141 MiB VRAM.
- Sistem belleği doğrulandı: 15,7 GB.
- İşlemci doğrulandı: Intel Core i5-12450H, 8 çekirdek / 12 iş parçacığı.
- Universal Robots UR10e resmî description paketi 4.3.1 sürümüne ve
  `ae333289875f9ba5a9ea6649a54036efb5ccabee` commit'ine kilitlendi.
- CoppeliaSim ZeroMQ istemcisi için Python geliştirme ortamı kuruldu.
- Hücre, fizik, üç robot, kameralar, kutular ve bant konfigürasyonları eklendi.
- Koli kimliği tabanlı görev koordinatörü yazıldı.
- Düşük güvenli sınıflandırmalar için reject davranışı yazıldı.
- Encoder/hız tabanlı varış zamanı hesabı yazıldı.
- EPick vakum alanı, normal/yanal kapasite ve moment kopma hesabı yazıldı.
- Gerçek ölçülü kutu atalet hesabı yazıldı.
- ROS 2 `BoxObservation`, `BoxTask` ve `PickBox` arayüzleri oluşturuldu.
- CoppeliaSim'de zemin, konveyör, yönlendirme platformu, üç sepet, ana kamera
  ve üretici URDF'sinden üç UR10e kurabilen sahne oluşturucu yazıldı.

## Test sonucu

```text
11 passed in 0.27s
```

Test kapsamı:

- Konfigürasyon güvenlik kuralları
- Ana/yerel kamera karar yetkisi
- Kırmızı ve mavi robot ataması
- Düşük güvenli etiketin reject edilmesi
- Tekrar gözlemde koli sahibinin değişmemesi
- Rezervasyon sonrası renk değişiminin reddedilmesi
- Görev durum makinesi
- Varış zamanına göre görev sıralama
- Kutu atalet hesabı
- Normal EPick tutuşu
- Kötü conta/aşırı yükte fiziksel tutuş kopması

## Tespit edilen ortam engelleri

- CoppeliaSim henüz kurulu değil.
- Mevcut WSL dağıtımı Ubuntu 26.04; hedef ROS 2 Jazzy ortamı Ubuntu 24.04 olmalı.
- WSL şu anda 7,6 GB RAM görüyor. Bu nedenle CoppeliaSim'in Windows'ta,
  ROS 2'nin ayrı Ubuntu 24.04 WSL dağıtımında geliştirilmesi veya native Ubuntu
  dual boot kullanılması gerekiyor.
- CoppeliaSim indirmeden önce Edu/Pro lisans uygunluğu belirlenmeli. Edu sürümü
  yalnız resmî lisans koşullarına uygun eğitim kullanımı içindir.

## Sıradaki teknik kapı

1. Lisans türünü belirlemek.
2. CoppeliaSim 4.10'u kurmak.
3. Ayrı Ubuntu 24.04 + ROS 2 Jazzy ortamını hazırlamak.
4. Resmî UR10e Xacro dosyasından sabit URDF üretmek.
5. İlk üç robotlu `.ttt` sahnesini oluşturmak ve 5 ms MuJoCo performansını ölçmek.

