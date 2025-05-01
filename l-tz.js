// Вспомогательные функции для перевода градусов в радианы и наоборот
function deg2rad(degrees) {
    return degrees * Math.PI / 180;
}

function rad2deg(radians) {
    return radians * 180 / Math.PI;
}

// Константы и входные данные
const EARTH_RADIUS = 6371000; // Радиус Земли в метрах
const SCALE = 0.38; // метры на пиксель

const droneAzimuthDeg = 335;
const controlPointLat = 50.603694;
const controlPointLon = 30.650625;
const imageCenterXpx = 320;
const imageCenterYpx = 256;
const controlPointXpx = 558;
const controlPointYpx = 328;

// 1. Вектор смещения в пикселях
const deltaXpx = controlPointXpx - imageCenterXpx; // 238
const deltaYpx = controlPointYpx - imageCenterYpx; // 72 (Y вниз)

// 2. Смещение в метрах
const deltaXm = deltaXpx * SCALE; // 90.44
const deltaYm = deltaYpx * SCALE; // 27.36

// 3. Расстояние в метрах
const distanceM = Math.sqrt(deltaXm * deltaXm + deltaYm * deltaYm); // ~94.49

// 4. Угол смещения относительно "верха" изображения (по часовой стрелке)
// Используем atan2(dX, -dY) для получения угла от "верха" (ось -Y)
const angleRelativeToImageUpRad = Math.atan2(deltaXpx, -deltaYpx);
let angleRelativeToImageUpDeg = rad2deg(angleRelativeToImageUpRad);
// Нормализуем, если нужно, хотя atan2 дает правильный квадрант
angleRelativeToImageUpDeg = (angleRelativeToImageUpDeg + 360) % 360; // ~106.83 deg

// 5. Абсолютный азимут от центра к контрольной точке
let absoluteAzimuthCenterToPointDeg = (droneAzimuthDeg + angleRelativeToImageUpDeg) % 360; // ~81.83 deg

// 6. Азимут от контрольной точки к центру (обратный)
const bearingPointToCenterDeg = (absoluteAzimuthCenterToPointDeg + 180) % 360; // ~261.83 deg

// 7. Вычисление координат центра
const lat1Rad = deg2rad(controlPointLat);
const lon1Rad = deg2rad(controlPointLon);
const bearingRad = deg2rad(bearingPointToCenterDeg);
const angularDistance = distanceM / EARTH_RADIUS; // ~0.00001483

// Формула для широты назначения
const lat2Rad = Math.asin(Math.sin(lat1Rad) * Math.cos(angularDistance) +
    Math.cos(lat1Rad) * Math.sin(angularDistance) * Math.cos(bearingRad));

// Формула для долготы назначения
const lon2Rad = lon1Rad + Math.atan2(Math.sin(bearingRad) * Math.sin(angularDistance) * Math.cos(lat1Rad),
    Math.cos(angularDistance) - Math.sin(lat1Rad) * Math.sin(lat2Rad));

// Перевод результата в градусы
const centerLatDeg = rad2deg(lat2Rad);
const centerLonDeg = rad2deg(lon2Rad);

// Вывод результата
console.log(`Расстояние от центра до контр. точки: ${distanceM.toFixed(2)} м`);
console.log(`Азимут от центра до контр. точки: ${absoluteAzimuthCenterToPointDeg.toFixed(2)}°`);
console.log(`Азимут от контр. точки до центра: ${bearingPointToCenterDeg.toFixed(2)}°`);
console.log('--- Координаты центра изображения ---');
console.log(`Широта: ${centerLatDeg.toFixed(6)}`); // ~50.603573
console.log(`Долгота: ${centerLonDeg.toFixed(6)}`); // ~30.649300

// Форматированный вывод для копирования
console.log(`\nКоординаты центра: ${centerLatDeg.toFixed(6)}, ${centerLonDeg.toFixed(6)}`);