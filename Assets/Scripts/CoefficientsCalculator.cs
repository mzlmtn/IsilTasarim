using DotNetMatrix;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CoefficientsCalculator : MonoBehaviour
{
    // Görsellerin çizilebilmesi için gerekli objeler:
    public GameObject PointObject, SurfaceLineObject, CamberLineObject, CLParent, USParent, LSParent, 
        NParent, AngleOfAttackObject;
    // Kullanıcıdan istenecek girdilerin arayüz objeleri:
    public InputField INPTNACACode, INPTPanelCount, INPTAOA, INPTAirSpeed;
    public Text TXTCLs;

    // Hesaplama işleminin durumunu kontrol eden bool değişken:
    public bool Calculate;

    // NACA kodu:
    public string NACACode;
    // Panel sayısı
    public int Nums;
    // Düzgün rejim hava hızı:
    public float AirSpeed;
    // Dalış açısı:
    public float AngleOfAttack;
    // Pi sayısı:
    public float PI = 4.0f * Mathf.Atan(1.0f);

    // K_ij integrallerini depolayan 2 boyutlu dizi:
    public float[][] Matris;
    // Sağ taraf vektörü:
    public float[] Vector;
    // CL hesabında kullanılacak hız, basınç, normal ve eksenel kuvvetleri depolayan diziler:
    public float[] VelocityVector, PressureVector, NormalForces, AxialForces;

    // Panelleri depolayan liste:
    public List<Panel> Panels = new List<Panel>();
    // Camber çizgisinin objelerini depolayan liste:
    private List<Panel> CamberPanels = new List<Panel>();

    // Lineer cebirsel denklem sistemini çözen kütüphane:
    private GeneralMatrix Gammas;

    // Çözümün tetiklendiği fonksiyon:
    public void Solve()
    {
        // Ok objesinin açısının dalış açısına getirilmesi:
        AngleOfAttackObject.transform.localEulerAngles = new Vector3(0, 0, AngleOfAttack);

        // Panel sayısı ataması:
        int newNum = Nums * 2 - 1;

        // Tüm 1 ve 2 boyutlu dizilerin tanımlanması:
        Matris = new float[newNum][];
        Vector = new float[newNum];
        VelocityVector = new float[newNum];
        PressureVector = new float[newNum];
        NormalForces = new float[newNum];
        AxialForces = new float[newNum];

        // Tüm 1 ve 2 boyutlu dizilerin depoladıkları sayıların 0'a eşitlenmesi:
        for (int i = 0; i < newNum; i++)
        {
            Matris[i] = new float[newNum];
            for (int j = 0; j < newNum; j++)
            {
                Matris[i][j] = 0;
            }
            Vector[i] = 0;
            VelocityVector[i] = 0;
            PressureVector[i] = 0;
            NormalForces[i] = 0;
            AxialForces[i] = 0;
        }

        // Tüm panelleri 1 kere tekrarlayan 1. döngü (i):
        for (int i = 0; i < newNum; i++)
        {
            // Sağ taraf vektörünün tanımlanması:
            float RHS = 0;

            // Tüm panelleri 1 kere tekrarlayan 2. döngü (j):
            for (int j = 0; j < newNum; j++)
            {
                // j. panelin uzunluğu ve K_ij değişkenlerinin tanımlanması:
                float sj = Panels[j].Length, K_ij = 0;

                // K_ij integrali için A değişkeninin hesabı:
                float A = -(Panels[i].CPX - Panels[j].SX) * cos(Panels[j].Angle) - (Panels[i].CPY - Panels[j].SY) * sin(Panels[j].Angle);
                
                // K_ij integrali için B değişkeninin hesabı:
                float B = pow(Panels[i].CPX - Panels[j].SX, 2) + pow(Panels[i].CPY - Panels[j].SY, 2);
                
                // K_ij integrali için C değişkeninin hesabı:
                float C = -cos(Panels[i].Angle - Panels[j].Angle);
                
                // K_ij integrali için D değişkeninin hesabı:
                float D = (Panels[i].CPX - Panels[j].SX) * cos(Panels[i].Angle) + (Panels[i].CPY - Panels[j].SY) * sin(Panels[i].Angle);
                
                // K_ij integrali için E değişkeninin hesabı:
                float E = pow(B - A * A, 0.5f);

                // Kökün içinin negatif olması durumunda işlemlerin devam edebilmesi için önlem:
                if (B - A * A < 0 || float.IsNaN(E) || float.IsInfinity(E))
                {
                    K_ij = 0;
                }
                // Kökün içinin negatif olmama durumu:
                else
                {
                    // K_ij değişkeninin hesabı:
                    K_ij = C * 0.5f * ln((sj * sj + 2 * A * sj + B) / B) + (D - A * C) *
                        (Mathf.Atan2((sj + A), E) - Mathf.Atan2(A, E)) / E;

                    // K_ij'nin hesaplanmamasını gerektirecek durumlar:
                    if (i == j || float.IsNaN(K_ij) || float.IsInfinity(K_ij))
                    {
                        K_ij = 0;
                    }

                    // i. panelin normaliyle dalış açısı arasındaki açının (beta) hesaplanması:
                    float beta = Panels[i].AngleOfNormal - AngleOfAttack * Mathf.Deg2Rad;
                    // Beta açısının (0,2*PI] aralığında tutulması:
                    if (beta > 2 * PI) beta -= 2 * PI;

                    // Kutta koşulu gereği matrisin son satırının ilk ve son sütunları haricinde 0'a,
                    // ilk ve son sütunlarının da 1'e eşitlenmesi gerekir.
                    // Eğer matrisin son satırına gelinmişse sağ taraf vektörünün 0 olması gerekir.

                    // Döngünün son satırda olup olmadığını kontrol etme:
                    if (i < newNum - 1)
                    {
                        // Son satıra gelinmemişse sağ taraf vektörü normal şekilde hesaplanır
                        RHS = -AirSpeed * 2 * PI * cos(beta);
                    }
                    else if (i == newNum - 1)
                    {
                        // Son satırın ilk ve son sütunlarının 1 olması için burada -1'e eşitlenir
                        // Çünkü aşağıda 'Matris[i][j] = -K_ij;' satırında iki -1 çarpımı +1 eder
                        if (j == 0 || j == newNum - 1)
                        {
                            K_ij = -1;
                        }
                        // Son satırın ilk ve son sütunları haricindeki elemanların 0'a eşitlenmesi:
                        else
                        {
                            K_ij = 0;
                        }
                        // Son satırda sağ taraf vektörünün 0'a eşitlenmesi:
                        RHS = 0;
                    }
                }
                
                // Hesaplanan K_ij'nin matrise yerleştirilmesi:
                Matris[i][j] = -K_ij;
                // Sağ taraf vektörünün yerleştirilmesi:
                Vector[i] = RHS;
            }
        }
        
        // Yukarıda oluşturulan matris ve vektörün aynısının matris çözücü kütüphanede oluşturulması:
        GeneralMatrix _A = new GeneralMatrix(Matris);
        GeneralMatrix _b = new GeneralMatrix(Vector.Length, 1);
        
        // Kütüphanede oluşturulan vektörün değerlerinin atanması:
        for (int i = 0; i < Vector.Length; i++)
        {
            _b.Array[i][0] = Vector[i];
        }

        // Matris ve vektörün kullanıcı için konsola yazdırılması:
        Debug.Log("A Matrix:\n" + Matrix.DebugMatrix(_A.Array, "|"));
        Debug.Log("b Matrix:\n" + Matrix.DebugMatrix(_b.Array));


        // Cebirsel denklem sisteminin kütüphane yardımıyla çözülmesi:
        Gammas = _A.Solve(_b);
        

        // Bilinmeyenlerin (gamma) kullanıcı için konsola yazdırılması:
        Debug.Log("Gammas Vector:\n" + Matrix.DebugMatrix(Gammas.Array));

        // CL'nin bulunması için gerekli 3 geçici değişken tanımı:
        float gammaSum = 0, CNSum = 0, CASum = 0;


        // L_ij hesabı için tüm panelleri 1 kere tekrarlayan 1. döngü (i):
        for (int i = 0; i < newNum; i++)
        {
            // i. panel kontrol noktasındaki teğetsel hızların toplamını depolayacak değişkenin tanımı:
            float velocitySum = 0;

            // Tüm panellerin gamma değerlerinin, o panelin uzunluğuyla çarpımının toplamı:
            gammaSum += Panels[i].Length * Gammas.Array[i][0];

            // L_ij hesabı için tüm panelleri 1 kere tekrarlayan 2. döngü (j):
            for (int j = 0; j < newNum; j++)
            {
                // j. panelin uzunluğu ve K_ij değişkenlerinin tanımlanması:
                float sj = Panels[j].Length, L_ij = 0;

                // L_ij integrali için A değişkeninin hesabı:
                float A = -(Panels[i].CPX - Panels[j].SX) * cos(Panels[j].Angle) - (Panels[i].CPY - Panels[j].SY) * sin(Panels[j].Angle);
                
                // L_ij integrali için B değişkeninin hesabı:
                float B = pow(Panels[i].CPX - Panels[j].SX, 2) + pow(Panels[i].CPY - Panels[j].SY, 2);

                // L_ij integrali için C değişkeninin hesabı:
                float C = sin(Panels[j].Angle - Panels[i].Angle);

                // L_ij integrali için D değişkeninin hesabı:
                float D = (Panels[i].CPX - Panels[j].SX) * sin(Panels[i].Angle) - (Panels[i].CPY - Panels[j].SY) * cos(Panels[i].Angle);

                // L_ij integrali için E değişkeninin hesabı:
                float E = pow(B - A * A, 0.5f);


                // Kökün içinin negatif olması durumunda işlemlerin devam edebilmesi için önlem:
                if (B - A * A < 0 || float.IsNaN(E) || float.IsInfinity(E))
                {
                    L_ij = 0;
                }
                // Kökün içinin negatif olmama durumu:
                else
                {
                    // L_ij değişkeninin hesabı:
                    L_ij = C * 0.5f * ln((sj * sj + 2 * A * sj + B) / B) + (D - A * C) *
                        (Mathf.Atan2((sj + A), E) - Mathf.Atan2(A, E)) / E;

                    // L_ij'nin hesaplanmamasını gerektirecek durumlar:
                    if (i == j || float.IsNaN(L_ij) || float.IsInfinity(L_ij))
                    {
                        L_ij = 0;
                    }
                }

                // i.panelin kontrol noktasındaki teğetsel hızı oluşturan 3 terimden 2'ncisinin
                // (denklem 2.53) hesaplanması:
                velocitySum = velocitySum - Gammas.Array[j][0] * L_ij / 2 / PI;
            }

            // i. panelin normaliyle dalış açısı arasındaki açının (beta) hesaplanması:
            float beta = Panels[i].AngleOfNormal - AngleOfAttack * Mathf.Deg2Rad;
            // Beta açısının (0,2*PI] aralığında tutulması:
            if (beta > 2 * PI) beta -= 2 * PI;

            // i. panelin kontrol noktasındaki teğetsel hızın hesaplanması:
            VelocityVector[i] = AirSpeed * sin(beta) + velocitySum + Gammas.Array[i][0] / 2;

            // i. panelin kontrol noktasındaki basıncın hesaplanması:
            PressureVector[i] = 1 - pow(VelocityVector[i] / AirSpeed, 2);

            // i. panelin kontrol noktasındaki normal kuvvetin hesaplanması:
            NormalForces[i] = -PressureVector[i] * Panels[i].Length * sin(beta);

            // i. panelin kontrol noktasındaki eksenel kuvvetin hesaplanması:
            AxialForces[i] = -PressureVector[i] * Panels[i].Length * cos(beta);


            // Toplam normal kuvvetin hesaplanması:
            CNSum += NormalForces[i] * cos(AngleOfAttack * Mathf.Deg2Rad);

            // Toplam eksenel kuvvetin hesaplanması:
            CASum += AxialForces[i] * sin(AngleOfAttack * Mathf.Deg2Rad);
        }

        // Her bir panelin kontrol noktasındaki hız ve basınç vektörlerinin kullanıcı için konsola
        // yazdırılması:
        Debug.Log("Velocities Vector:\n" + Matrix.DebugVector(VelocityVector));
        Debug.Log("Pressures Vector:\n" + Matrix.DebugVector(PressureVector));


        // CL'nin ilk hesaplanma yöntemi:
        float cl1 = CNSum - CASum;

        // CL'nin ikinci hesaplanma yöntemi:
        float cl2 = 2 * gammaSum / AirSpeed;


        // CL değerlerinin kullanıcı için konsola yazdırılması:
        Debug.Log("CL1:" + cl1 + " CL2:" + cl2);

        // CL değerlerinin kullanıcı için ekrana yazdırılması:
        TXTCLs.text = "CL1:[" + cl1 + "] CL2:[" + cl2 + "]";
    }


    // NACA MPXX
    // M is the maximum camber divided by 100.
    // P is the position of the maximum camber divided by 10.
    // XX is the thickness divided by 100.

    // 4 basamaklı NACA kodunun tanımlanması için gereken 3 sayı:
    public float M, P, XX;
    // Ekranda yaratılan objelerin depolanacağı diziler:
    public GameObject[] CamberLObjects, UpperSObjects, LowerSObjects;
    // Kanadı çizen fonksiyon:
    public void NACA4AirfoilCreator(string code, int num) 
    {
        // Kodun 4 basamak olduğunu doğrulama:
        if (code.Length == 4)
        {
            // Okların açısını dalış açısına getirme:
            AngleOfAttackObject.transform.localEulerAngles = new Vector3(0, 0, AngleOfAttack);

            // kullanıcı tarafından verilen 4 basamaklı girdiyi parçalayıp doğru değerlere atama:
            M = float.Parse(code[0].ToString()) / 100;
            P = float.Parse(code[1].ToString()) / 10;
            XX = float.Parse((code[2].ToString() + code[3].ToString()).ToString()) / 100;

            // Önceden çizilmiş bir kanat varsa objelerini silme:
            for (int i = 0; i < CamberLObjects.Length; i++)
            {
                Destroy(CamberLObjects[i]);
                Destroy(UpperSObjects[i]);
                Destroy(LowerSObjects[i]);
            }
            // Sıfırlanan dizileri yeni girilen panel sayısına göre boyutlandırma:
            CamberLObjects = new GameObject[num];
            UpperSObjects = new GameObject[num];
            LowerSObjects = new GameObject[num];

            // CL değerinin daha iyi hesaplanması için panel başlangıç ve bitiş noktalarının daha iyi 
            // konumlandırılması gerekir. Bunu cosine fonksiyonu yapabilir

            // Cosine fonksiyonu için açı değeri
            float beta = 0;
            // Cosine fonksiyonu için açının her bir adımda artış değeri:
            float angleInc = PI / num;
            // Panel sayısına kadar giden döngü oluşturulması:
            for (int i = 0; i < num; i++)
            {
                // beta açısına göre camber çizgisinin noktasının x koordinatının cosine fonksiyonu
                // yardımıyla çizilmesi:
                float x = (1 - Mathf.Cos(beta)) / 2;
                // beta açısının arttırılması:
                beta += angleInc;
                // beta açısına göre camber çizgisinin noktasının y koordinatının cosine fonksiyonu
                // yardımıyla çizilmesi:
                float y = CamberPositionY(x);

                // Camber çizgisinin noktasının objesinin yaratılması:
                GameObject CamberLine = Instantiate(PointObject, new Vector3(x, y, 0), Quaternion.identity);
                // Camber objesinin noktasının isminin düzenlenmesi:
                CamberLine.name = "CL: " + i;
                // Camber çizgilerinin depolandığı diziye yeni yaratılan camber çizgisi nokta
                // objesinin eklenmesi:
                CamberLObjects[i] = CamberLine;
                CamberLine.transform.SetParent(CLParent.transform);

                // Camber çizgisinin (x, y) noktasındaki teğetinin eğiminin hesaplanması:
                float theta = Mathf.Atan(CamberSlope(x));
                // Kanat geometrisinin camber çizgisinden uzaklığı:
                float yt = FoilThickness(x);

                // Kanadın üst kısmının noktalarının x ve y koordinatlarının hesaplanması:
                float xu = x - yt * Mathf.Sin(theta);
                float yu = y + yt * Mathf.Cos(theta);
                // Kanadın alt kısmının noktalarının x ve y koordinatlarının hesaplanması:
                float xl = x + yt * Mathf.Sin(theta);
                float yl = y - yt * Mathf.Cos(theta);

                // Kanadın üst ve alt kısımlarının noktalarının objelerinin yaratılması:
                GameObject UpperSurface = Instantiate(PointObject, new Vector3(xu, yu, 0), Quaternion.identity);
                GameObject LowerSurface = Instantiate(PointObject, new Vector3(xl, yl, 0), Quaternion.identity);
                UpperSurface.transform.SetParent(USParent.transform);
                LowerSurface.transform.SetParent(LSParent.transform);

                // Kanadın üst ve alt kısımlarının noktalarının isimlerinin düzenlenmesi:
                UpperSurface.name = "US: " + i;
                LowerSurface.name = "LS: " + i;

                // Kanadın üst ve alt kısımlarının nokta objelerinin depolandığı diziye yeni yaratılan
                // kanat objelerinin eklenmesi:
                UpperSObjects[i] = UpperSurface;
                LowerSObjects[i] = LowerSurface;
            }

            // Çizgilerin yaratılmasını tetikleyen fonksiyon:
            CreatePanels();
            // Çizgiler yaratıldıktan sonra çözümü başlatan fonksiyon:
            Solve();
        }
    }
    // Camber çizgisinin koordinatlarını analitik olarak veren fonksiyon:
    public float CamberPositionY(float x)
    {
        if (x >= 0 && x < P)
        {
            return M * (2 * P * x - x * x) / P / P;
        }
        else if (x >= P && x <= 1)
        {
            return M * (1 - 2 * P + 2 * P * x - x * x) / Mathf.Pow(1 - P, 2);
        }
        else return 0;
    }
    // Camber çizgisinin teğetinin eğimini analitik olarak veren fonksiyon:
    public float CamberSlope(float x)
    {
        if (x >= 0 && x < P)
        {
            return 2 * M * (P - x) / P / P;
        }
        else if (x >= P && x <= 1)
        {
            return 2 * M * (P - x) / Mathf.Pow(1 - P, 2);
        }
        else return 0;
    }
    // Kanadın kalınlığını veren fonksiyon
    // (camber çizgisi kanadın üst ve alt yüzeyine eşit uzaklıktadır):
    public float FoilThickness(float x)
    {
        if (x >= 0 && x <= 1)
        {
            float a0 = 0.2969f, a1 = -0.126f, a2 = -0.3516f, a3 = 0.2843f, a4 = -0.1036f;
            return XX * (a0 * Mathf.Pow(x, 0.5f) + a1 * x + a2 * x * x + a3 * x * x * x + a4 * x * x * x * x) / 0.2f;
        }
        else return 0;
    }

    // Kanadın üst ve alt yüzey çizgilerinin objelerini, camber çizgisinin objelerini ve "Panel"
    // objelerinin yaratılması:
    public void CreatePanels()
    {
        // Önceden yaratılmış objeleri ve Panel objelerinin sıfırlanması:
        for (int i = 0; i < Panels.Count; i++)
        {
            Destroy(Panels[i].PanelObject);
            Destroy(Panels[i].NormalObject);
        }
        Panels.Clear();
        for (int i = 0; i < CamberPanels.Count; i++)
        {
            Destroy(CamberPanels[i].PanelObject);
        }
        CamberPanels.Clear();

        // Geçici değişken sayaç:
        int counter = 0;

        // Kanadın alt ve üst yüzeylerinin çizgi objelerinin yaratılması:
        // Panel objelerinin yaratılması:
        for (int i = LowerSObjects.Length - 1; i >= 1; i--)
        {
            Panel p = new Panel(LowerSObjects[i].transform.position.x, LowerSObjects[i].transform.position.y,
                LowerSObjects[i - 1].transform.position.x, LowerSObjects[i - 1].transform.position.y);
            GameObject go = DrawLine(p.SX, p.SY, p.EX, p.EY, SurfaceLineObject);
            counter++;
            go.name = "GreenSurface:" + counter;
            p.PanelObject = go;
            Panels.Add(p);
        }

        Panel headingPanel = new Panel(LowerSObjects[0].transform.position.x, LowerSObjects[0].transform.position.y,
            UpperSObjects[1].transform.position.x, UpperSObjects[1].transform.position.y);
        GameObject hgo = DrawLine(headingPanel.SX, headingPanel.SY, headingPanel.EX, headingPanel.EY, SurfaceLineObject);
        counter++;
        hgo.name = "GreenSurface:" + counter;
        headingPanel.PanelObject = hgo;
        Panels.Add(headingPanel);

        for (int i = 1; i < UpperSObjects.Length - 1; i++)
        {
            Panel p = new Panel(UpperSObjects[i].transform.position.x, UpperSObjects[i].transform.position.y,
                UpperSObjects[i + 1].transform.position.x, UpperSObjects[i + 1].transform.position.y);
            GameObject go = DrawLine(p.SX, p.SY, p.EX, p.EY, SurfaceLineObject);
            counter++;
            go.name = "GreenSurface:" + counter;
            p.PanelObject = go;
            Panels.Add(p);
        }
        /*
        Panel trailingPanel = new Panel(UpperSObjects[UpperSObjects.Length - 1].transform.position.x, UpperSObjects[UpperSObjects.Length - 1].transform.position.y,
            LowerSObjects[LowerSObjects.Length - 1].transform.position.x, LowerSObjects[LowerSObjects.Length - 1].transform.position.y);
        */
        Panel trailingPanel = new Panel(UpperSObjects[UpperSObjects.Length - 1].transform.position.x, UpperSObjects[UpperSObjects.Length - 1].transform.position.y,
            Panels[Panels.Count - 1].Length / 2 + 1, 0);

        GameObject tgo = DrawLine(trailingPanel.SX, trailingPanel.SY, trailingPanel.EX, trailingPanel.EY, SurfaceLineObject);
        counter++;
        tgo.name = "GreenSurface:" + counter;
        trailingPanel.PanelObject = tgo;
        Panels.Add(trailingPanel);

        // Paneller saat yönünde dizilecek şekilde yaratıldı, dizilişi saatin tersi yönüne çevirme:
        Panels.Reverse();

        // Camber çizgisinin çizgi objelerinin çizilmesi:
        for (int i = 0; i < CamberLObjects.Length - 1; i++)
        {
            Panel p = new Panel(CamberLObjects[i].transform.position.x, CamberLObjects[i].transform.position.y,
                CamberLObjects[i + 1].transform.position.x, CamberLObjects[i + 1].transform.position.y);
            GameObject clgo = DrawLine(p.SX, p.SY, p.EX, p.EY, CamberLineObject);
            clgo.transform.localScale = new Vector3(clgo.transform.localScale.x, 0.0075f, 0.0075f);
            clgo.transform.SetParent(CLParent.transform);
            p.PanelObject = clgo;
            CamberPanels.Add(p);
        }

        // Her bir panelin kontrol noktasından, o panelin normalini çizdirme:
        for (int i = 0; i < Panels.Count; i++)
        {
            GameObject normal = DrawLine(Panels[i].CPX, Panels[i].CPY, Panels[i].EndPointOfNormalX, Panels[i].EndPointOfNormalY, PointObject);
            normal.transform.localScale = new Vector3(normal.transform.localScale.x, 0.003f, 0.003f);
            normal.transform.SetParent(NParent.transform);
            Panels[i].NormalObject = normal;
        }
    }



    // Bir objeyi doğru parçası haline getirme (verilenler: başlangıç noktası (x, y),
    // bitiş noktası (x, y), doğru parçası haline getirilecek obje):
    public GameObject DrawLine(float sx, float sy, float ex, float ey, GameObject go)
    {
        GameObject GO = Instantiate(go, Vector3.zero, Quaternion.identity);
        // pozisyon:
        GO.transform.position = new Vector3((sx + ex) / 2, (sy + ey) / 2, 0);
        // açı:
        GO.transform.rotation = Quaternion.Euler(0, 0, Mathf.Atan2((ey - sy), (ex - sx)) * Mathf.Rad2Deg);
        // büyüklük:
        GO.transform.localScale = new Vector3(Mathf.Sqrt(Mathf.Pow(sx - ex, 2) + Mathf.Pow(sy - ey, 2)), 0.01f, 0.01f);
        
        return GO;
    }

    // Saniyedeki kare sayısı kadar kere otomatik olarak çağırılan fonksiyon:
    void Update()
    {
        // Hesaplama işleminin durumunu kontrol etme:
        if (Calculate)
        {
            Calculate = false;

            // Kullanıcının girdiği değerleri çekme:
            NACACode = INPTNACACode.text;
            Nums = int.Parse(INPTPanelCount.text) / 2;
            AngleOfAttack = float.Parse(INPTAOA.text);
            AirSpeed = float.Parse(INPTAirSpeed.text);

            // Kanadı yaratan fonksiyonu çağırma:
            NACA4AirfoilCreator(NACACode, Nums);
        }
    }

    // Hesapla butonuna basıldığında çalışan fonksiyon:
    public void BTNCalculate()
    {
        Calculate = true;
        if (Calculate)
        {
            Calculate = false;

            // Kullanıcının girdiği değerleri çekme:
            NACACode = INPTNACACode.text;
            Nums = int.Parse(INPTPanelCount.text) / 2;
            AngleOfAttack = float.Parse(INPTAOA.text);
            AirSpeed = float.Parse(INPTAirSpeed.text);

            // Kanadı yaratan fonksiyonu çağırma:
            NACA4AirfoilCreator(NACACode, Nums);
        }
    }

    // Bazı matematiksel fonksiyonların yazarken kolaylık sağlaması için kısaltılması:
    #region Equation Shortcuts
    public float cos(float x)
    {
        return Mathf.Cos(x);
    }
    public float sin(float x)
    {
        return Mathf.Sin(x);
    }
    public float tan(float x)
    {
        return Mathf.Tan(x);
    }
    public float atan(float x)
    {
        return Mathf.Atan(x);
    }
    public float ln(float x)
    {
        return Mathf.Log(x);
    }
    public float pow(float x, float y)
    {
        return Mathf.Pow(x, y);
    }
    public float log(float x, float y)
    {
        return Mathf.Log(x, y);
    }
    #endregion
}

// Panel nesnesinin tanımlanması:
[System.Serializable]
public class Panel
{
    // Panelin başlangıç noktalarının koordinatları:
    public float SX, SY;
    // Panelin bitiş noktalarının koordinatları
    public float EX, EY;
    // Panelin kontrol noktalarının koordinatları
    public float CPX, CPY;

    // Panelin uzunluğu:
    public float Length;
    // Panelin +x ekseniyle yaptığı açı ve normalinin açısı:
    public float Angle, AngleOfNormal;
    // Panelin çizilecek olan normalinin bitiş koordinatları:
    public float EndPointOfNormalX, EndPointOfNormalY;
    // Panelin panel ve normal objeleri:
    public GameObject PanelObject, NormalObject;

    // Panel nesnesinin Constructor'u:
    public Panel(float sx, float sy, float ex, float ey, float normalLength = 0.15f)
    {
        // Gelen değerlerin atanması/hesaplanması:
        SX = sx;
        SY = sy;
        EX = ex;
        EY = ey;
        CPX = (sx + ex) / 2;
        CPY = (sy + ey) / 2;
        Length = Mathf.Sqrt(Mathf.Pow(SX - EX, 2) + Mathf.Pow(SY - EY, 2));
        Angle = Mathf.Atan2((EY - SY), (EX - SX));
        if (Angle < 0) Angle += Mathf.PI * 2;

        AngleOfNormal = Angle + Mathf.PI / 2;
        if (AngleOfNormal > 2 * Mathf.PI)
            AngleOfNormal -= 2 * Mathf.PI;

        // Aşağıda normalin çizilmesi için geometrik işlemler yer alır.
        // Başlangıç noktası, eğimi ve uzunluğu bilinen bir doğru parçasının bitiş noktası bulunur:

        // A(a, b) = A(CPX, CPY) -> Başlangıç noktası kontrol noktasıdır.
        // m -> Normalin eğimidir.
        // l=2 -> Normalin uzunluğudur, değiştirilebilir.
        // B(c, d) -> Bulunmak istenen bitiş noktasıdır.
        // y-b=m*(x-a) -> Normalin doğru denklemi.
        // (1+m*m)*c^2 - (2*a+2*m*m*a)*c + m*m*a*a+a*a-l*l = 0 -> Gerekli işlemler yapıldıktan sonra c 
        // koordinatına bağlı (normalin bitiş noktasının x koordinatı, bilinmeyen) 2. dereceden bir 
        // denklem bulunur ve kökleri bulunabilir.

        // Eğim
        float m = Mathf.Tan(AngleOfNormal);
        // Başlangıç noktası:
        float a = CPX, b = CPY;
        //Uzunluk:
        float l = normalLength;

        // Delta bulunabilmesi için a, b ve c geçici değişkenlerinin tanımlanması:
        float _a = 1 + m * m, _b = -(2 * a + 2 * m * m * a), _c = m * m * a * a + a * a - l * l;

        // Delta hesaplanması:
        float delta = _b * _b - 4 * _a * _c;
        // İki farklı reel kök çıkabileceği için 2 farklı bitiş noktasının tanımlanması:
        float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        // Karmaşık kök yoksa:
        if (delta >= 0)
        {
            // 1. kökün x koordinatı
            x1 = (-_b + Mathf.Sqrt(delta)) / 2 / _a;
            // 2. kökün x koordinatı
            x2 = (-_b - Mathf.Sqrt(delta)) / 2 / _a;
        }
        // Denklemde x koordinatları yerlerine konarak 1. ve 2. köklerin y koordinatları bulunur:
        y1 = m * (x1 - a) + b;
        y2 = m * (x2 - a) + b;

        // Mantıklı kökün seçilmesi ve normalin bitiş noktası olarak atanması:
        if (ey < sy)
        {
            EndPointOfNormalX = x1;
            EndPointOfNormalY = y1;
        }
        else
        {
            EndPointOfNormalX = x2;
            EndPointOfNormalY = y2;
        }
    }
}


