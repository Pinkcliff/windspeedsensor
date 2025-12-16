import math
class ref:
    def __init__(self):
        pass
        #TODO
    def __DH__(self,**kwargs):
        pass
        #TODO



class AIR:
    CPDA = 0.24
    QEW = 597.3
    CPW = 0.441
    COEFF0622 = 0.622
    D = [-5674.5359,
         6.3925247,
         -0.009677843,
         0.00000062215701,
         2.0747825E-09,
         -9.484024E-13,
         4.1635019,
         -5800.2206,
         1.3914993,
         -0.048640239,
         0.000041764768,
         -0.000000014452039,
         6.5459673]

    EPS = 0.00003

    def __init__(self,dP,unit,**kwargs):
        self.flgTdb = False
        self.flgTwb = False
        self.flgTdp = False
        self.flgRh = False
        self.flgH = False
        self.flgX = False
        self.flgS = False
        self.flgV = False
        self.flgLambda = False
        self.flgDen = False
        if unit == 'k' or unit == 'K':
            self.dP = dP
            if 'dTdb' in kwargs:
                self.dTdb = kwargs.get('dTdb')
                self.flgTdb = True
            if 'dTwb' in kwargs:
                self.dTwb = kwargs.get('dTwb')
                self.flgTwb = True
            if 'dTdp' in kwargs:
                self.dTdp = kwargs.get('dTdp')
                self.flgTdp = True
            if 'dRh' in kwargs:
                self.dRh = kwargs.get('dRh')
                self.flgRh = True
            if 'dH' in kwargs:
                self.dH = kwargs.get('dH')
                self.flgH = True
            if 'dX' in kwargs:
                self.dX = kwargs.get('dX')
                self.flgX = True
            if 'dS' in kwargs:
                self.dS = kwargs.get('dS')
                self.flgS = True
            if 'dV' in kwargs:
                self.dV = kwargs.get('dV')
                self.flgV = True
            if 'dMu' in kwargs:
                self.dMu = kwargs.get('dMu')
                self.flgMu = True
            if 'dLambda' in kwargs:
                self.dLambda = kwargs.get('dLambda')
                self.flgLambda = True
            if 'Den' in kwargs:
                self.Den = kwargs.get('Den')
                self.flgDen = True
        if unit == 'c' or unit == 'C':
            self.dP = dP
            if 'dTdb' in kwargs:
                self.dTdb = kwargs.get('dTdb') + 273.15
                self.flgTdb = True
            if 'dTwb' in kwargs:
                self.dTwb = kwargs.get('dTwb') + 273.15
                self.flgTwb = True
            if 'dTdp' in kwargs:
                self.dTdp = kwargs.get('dTdp') + 273.15
                self.flgTdp = True
            if 'dRh' in kwargs:
                self.dRh = kwargs.get('dRh')
                self.flgRh = True
            if 'dH' in kwargs:
                self.dH = kwargs.get('dH')
                self.flgH = True
            if 'dX' in kwargs:
                self.dX = kwargs.get('dX')
                self.flgX = True
            if 'dS' in kwargs:
                self.dS = kwargs.get('dS')
                self.flgS = True
            if 'dV' in kwargs:
                self.dV = kwargs.get('dV')
                self.flgV = True
            if 'dMu' in kwargs:
                self.dMu = kwargs.get('dMu')
                self.flgMu = True
            if 'dLambda' in kwargs:
                self.dLambda = kwargs.get('dLambda')
                self.flgLambda = True
            if 'Den' in kwargs:
                self.Den = kwargs.get('Den')
                self.flgDen = True


    def FD__TdbTwb(dTdb,dTwb,dP):
        pwater = AIR.FPw_Sat(dTdb)
        Rh = AIR.FRh__TdbTwb(dTdb, dTwb, dP)
        pwater *= Rh
        # System.out.println("水蒸气分压："+pwater);
        pair = dP
        Den = 0.003484*pair/dTdb - 0.00134*pwater/dTdb
        Den *= 1000
        return Den

    # //新增 根据空气的干球温度和相对湿度对空气的密度进行计算
    def FD__TdbRh(dTdb, Rh, dP):
        dTwb = AIR.FTwb__TdbRh(dTdb, Rh, dP)
        Den = AIR.FD__TdbTwb(dTdb,dTwb,dP)
        return Den

    # //新增 根据空气的干球温度和绝对湿度对空气的密度进行计算
    def FD__TdbX(dTdb, X, dP):
        dTwb = AIR.FTwb__TdbX(dTdb, X, dP)
        Den = AIR.FD__TdbTwb(dTdb,dTwb,dP)
        return Den



    def FTdb__HX(dH, dX, dP):
        return (dH / 4.18605 - AIR.QEW * dX) / (AIR.CPDA + AIR.CPW * dX) + 273.15

     
    def FTwb__TdbTdp(dTdb, dTdp, dP):
        dX = AIR.FX__TdbTdp(dTdb, dTdp, dP)
        return AIR.FTwb__TdbX(dTdb, dX, dP)

     
    def FTwb__TdbX(dTdb, dX, dP):
        dTwb = 0.0  # wet-bulb temperature
        dT_Max = dTdb
        dT_Min = 273.15 - 200.0

        # partial pressure of vapor
        dPw = 0.0
        dPwd = AIR.FPw__X(dX, dP)
        dPw_Sat = AIR.FPw_Sat(dTdb)  # saturated vapor pressure
        if dPwd > dPw_Sat:
            dPwd = dPw_Sat  # get wet-bulb temperature

        while True:
            dTwb = (dT_Min + dT_Max) * 0.5
            dPw = AIR.FPw__TdbTwb(dTdb, dTwb, dP)

            if dPwd == 0.0:
                if abs(dPw) < AIR.EPS:
                    break
            elif abs(dPw / dPwd - 1.0) < AIR.EPS or abs(dT_Min - dT_Max) / (dT_Min + dT_Max) < AIR.EPS:
                break

            if dPw < dPwd:
                dT_Min = dTwb
            else:
                dT_Max = dTwb

        return dTwb

     
    def FTwb__TdbRh(dTdb, dRh, dP):
        dPw = AIR.FPw_Sat(dTdb) * dRh
        dX = AIR.COEFF0622 * dPw / (dP - dPw)
        dTwb = AIR.FTwb__TdbX(dTdb, dX, dP)

        return dTwb

     
    def FTwb__TdbH(dTdb, dH, dP):
        dX = AIR.FX__TdbH(dTdb, dH, dP)
        dTwb = AIR.FTwb__TdbX(dTdb, dX, dP)

        return dTwb

     
    def FTwb__HX(dH, dX, dP):
        dTdb = AIR.FTdb__HX(dH, dX, dP)
        dTwb = AIR.FTwb__TdbX(dTdb, dX, dP)
        return dTwb

     
    def FTdp__TdbX(dTdb, dX, dP):
        dT = 0.0  # Initialize dew-point temperature
        dT_Max = dTdb
        dT_Min = 273.15 - 200.0

        dPwd = AIR.FPw__X(dX, dP)  # partial pressure of vapor
        dPw_Sat = AIR.FPw_Sat(dTdb)  # saturated vapor pressure
        if dPwd > dPw_Sat:
            dPwd = dPw_Sat

        # Get dew-point temperature
        while True:
            dT = (dT_Min + dT_Max) * 0.5
            dPw_Sat = AIR.FPw_Sat(dT)

            if dPwd == 0.0:
                if abs(dPw_Sat) < AIR.EPS:
                    break
            elif abs(dPw_Sat / dPwd - 1.0) < AIR.EPS:
                break

            if dPw_Sat < dPwd:
                dT_Min = dT
            else:
                dT_Max = dT

            if abs(dT_Min - dT_Max) < AIR.EPS:
                break

        return dT

     
    def FTdp__TdbTwb(dTdb, dTwb, dP):
        dX = AIR.FX__TdbTwb(dTdb, dTwb, dP)
        return AIR.FTdp__TdbX(dTdb, dX, dP)

     
    def FTdp__TdbRh(dTdb, dRh, dP):
        dX = AIR.FX__TdbRh(dTdb, dRh, dP)
        return AIR.FTdp__TdbX(dTdb, dX, dP)

     
    def FTdp__TdbH(dTdb, dH, dP):
        dX = AIR.FX__TdbH(dTdb, dH, dP)
        return AIR.FTdp__TdbX(dTdb, dX, dP)

     
    def FTdp__HX(dH, dX, dP):
        dTdb = AIR.FTdb__HX(dH, dX, dP)
        return AIR.FTdp__TdbX(dTdb, dX, dP)

    def FRh__TdbTwb(dTdb, dTwb, dP):
        dPw = AIR.FPw__TdbTwb(dTdb, dTwb, dP)
        dPw_Sat = AIR.FPw_Sat(dTdb)
        if dPw > dPw_Sat:
            dPw = dPw_Sat
        return dPw / dPw_Sat

     
    def FRh__TdbTdp(dTdb, dTdp, dP):
        dTwb = AIR.FTwb__TdbTdp(dTdb, dTdp, dP)
        return AIR.FRh__TdbTwb(dTdb, dTwb, dP)

     
    def FRh__TdbH(dTdb, dH, dP):
        dTwb = AIR.FTwb__TdbH(dTdb, dH, dP)
        return AIR.FRh__TdbTwb(dTdb, dTwb, dP)

     
    def FRh__TdbX(dTdb, dX, dP):
        dPw = AIR.FPw__X(dX, dP)
        dPw_Sat = AIR.FPw_Sat(dTdb)
        if dPw > dPw_Sat:
            dPw = dPw_Sat
        return dPw / dPw_Sat

     
    def FRh__HX(dH, dX, dP):
        dTdb = AIR.FTdb__HX(dH, dX, dP)
        dTwb = AIR.FTwb__TdbX(dTdb, dX, dP)
        return AIR.FRh__TdbTwb(dTdb, dTwb, dP)

     
    def FH__TdbX(dTdb, dX, dP):
        # coefficients are in kcal-unit
        dH = (AIR.CPDA * (dTdb - 273.15) + (AIR.QEW + AIR.CPW * (dTdb - 273.15)) * dX)
        # re-translate to SI
        return dH * 4.18605

     
    def FH__TdbTwb(dTdb, dTwb, dP):
        dX = AIR.FX__TdbTwb(dTdb, dTwb, dP)
        return AIR.FH__TdbX(dTdb, dX, dP)

     
    def FH__TdbTdp(dTdb, dTdp, dP):
        dX = AIR.FX__TdbTdp(dTdb, dTdp, dP)
        return AIR.FH__TdbX(dTdb, dX, dP)

     
    def FH__TdbRh(dTdb, dRh, dP):
        dX = AIR.FX__TdbRh(dTdb, dRh, dP)
        return AIR.FH__TdbX(dTdb, dX, dP)

     
    def FX__TdbH(dTdb, dH, dP):
        # coefficients are in kcal-unit
        dX = (dH / 4.18605 - AIR.CPDA * (dTdb - 273.15)) / (AIR.QEW + AIR.CPW * (dTdb - 273.15))
        return dX

     
    def FX__TdbTwb(dTdb, dTwb, dP):
        dPw = AIR.FPw__TdbTwb(dTdb, dTwb, dP)
        return AIR.FX__Pw(dPw, dP)

     
    def FX__TdbTdp(dTdb, dTdp, dP):
        dPw = AIR.FPw_Sat(dTdp)
        return AIR.FX__Pw(dPw, dP)

     
    def FX__TdbRh(dTdb, dRh, dP):
        dPw = dRh * AIR.FPw_Sat(dTdb)
        return AIR.FX__Pw(dPw, dP)

     
    def FX__Pw(dPw, dP):
        return AIR.COEFF0622 * dPw / (dP - dPw)

     
    def FS__TdbTwb(dTdb, dTwb, dP):

        dS = 0.0  # Entropy

        # Partial pressure of vapor
        dPw = AIR.FPw__TdbTwb(dTdb, dTwb, dP)

        # Saturated vapor pressure
        dPw_Sat = AIR.FPw_Sat(273.15)

        # Absolute humidity
        dX = AIR.FX__Pw(dPw, dP)

        if dPw == 0.:
            dLNPw = 0.0
        else:
            dLNPw = math.log(dPw / dPw_Sat)

        # Entropy calculation
        dS = AIR.CPDA * math.log(dTdb / 273.15) - 0.0685726 * math.log(dP - dPw) + \
             dX * (AIR.CPW * math.log(dTdb / 273.15) - 0.110234 * dLNPw + AIR.QEW / 273.15)

        return dS * 4.18605

     
    def FS__TdbTdp(dTdb, dTdp, dP):
        # Assuming FTwb__TdbTdp is already   defined

        # Calculate wet-bulb temperature
        dTwb = AIR.FTwb__TdbTdp(dTdb, dTdp,dP)

        # Calculate entropy
        dS = AIR.FS__TdbTwb(dTdb, dTwb, dP)

        return dS

     
    def FS__TdbRh(dTdb, dRh, dP):
        # Assuming FTwb__TdbRh and FS__TdbTwb are already   defined

        # Calculate wet-bulb temperature
        dTwb = AIR.FTwb__TdbRh(dTdb, dRh, dP)

        # Calculate entropy
        dS = AIR.FS__TdbTwb(dTdb, dTwb, dP)

        return dS

     
    def FS__TdbH(dTdb, dH, dP):
        # Assuming FTwb__TdbH and FS__TdbTwb are already   defined

        # Calculate wet-bulb temperature
        dTwb = AIR.FTwb__TdbH(dTdb, dH, dP)

        # Calculate entropy
        dS = AIR.FS__TdbTwb(dTdb, dTwb, dP)

        return dS

     
    def FS__TdbX(dTdb, dX, dP):
        # Assuming FTwb__TdbX and FS__TdbTwb are already   defined

        # Calculate wet-bulb temperature
        dTwb = AIR.FTwb__TdbX(dTdb, dX, dP)

        # Calculate entropy
        dS = AIR.FS__TdbTwb(dTdb, dTwb, dP)

        return dS

     
    def FS__HX(dH, dX, dP):
        # Assuming FTdb__HX, FTwb__TdbX, and FS__TdbTwb are already   defined

        # Calculate dry-bulb temperature
        dTdb = AIR.FTdb__HX(dH, dX, dP)

        # Calculate wet-bulb temperature
        dTwb = AIR.FTwb__TdbX(dTdb, dX, dP)

        # Calculate entropy
        dS = AIR.FS__TdbTwb(dTdb, dTwb, dP)

        return dS

     
    def FV__TdbX(dTdb, dX, dP):
        return 0.461535 * (AIR.COEFF0622 + dX) * dTdb / dP

     
    def FV__TdbTwb(dTdb, dTwb, dP):
        dX = AIR.FX__TdbTwb(dTdb, dTwb, dP)
        return AIR.FV__TdbX(dTdb, dX, dP)

     
    def FV__TdbTdp(dTdb, dTdp, dP):
        dX = AIR.FX__TdbTdp(dTdb, dTdp, dP)
        return AIR.FV__TdbX(dTdb, dX, dP)

     
    def FV__TdbRh(dTdb, dRh, dP):
        dX = AIR.FX__TdbRh(dTdb, dRh, dP)
        return AIR.FV__TdbX(dTdb, dX, dP)

     
    def FV__TdbH(dTdb, dH, dP):
        dX = AIR.FX__TdbH(dTdb, dH, dP)
        return AIR.FV__TdbX(dTdb, dX, dP)

     
    def FV__HX(dH, dX, dP):
        dTdb = AIR.FTdb__HX(dH, dX, dP)
        return AIR.FV__TdbX(dTdb, dX, dP)

     
    def FPw__X(dX, dP):
        return dP * dX / (AIR.COEFF0622 + dX)

    def FPw__TdbTwb(dTdb, dTwb, dP):
        dHC = 0.0
        dX = 0.0
        dPw_Sat = AIR.FPw_Sat(dTwb)  # Assuming FPw_Sat is   defined

        dX_Sat = AIR.FX__Pw(dPw_Sat, dP)  # Assuming FX__Pw is   defined

        if dTwb <= 273.15:
            dHC = -79.7 + 0.5 * (dTwb - 273.15) + 273.15
        else:
            dHC = dTwb

        dX = dX_Sat - (AIR.CPDA + AIR.CPW * dX_Sat) * (dTdb - dTwb) / (
                AIR.QEW + AIR.CPW * (dTdb - 273.15) - (
                dHC - 273.15))  # Assuming CPDA, CPW, QEW are   defined
        if dX < 0.0:
            dX = 0

        return AIR.FPw__X(dX, dP)  # Assuming FPw__X is   defined


    def FPw_Sat(dTdb):

        if dTdb < 273.15:
            dLNP = AIR.D[0] / dTdb + AIR.D[1] + AIR.D[2] * dTdb + AIR.D[3] * pow(dTdb, 2) + AIR.D[4] * pow(dTdb, 3) \
                   + AIR.D[5] * pow(dTdb, 4) + AIR.D[6] * math.log(dTdb)
        else:
            dLNP = AIR.D[7] / dTdb + AIR.D[8] + AIR.D[9] * dTdb + AIR.D[10] * pow(dTdb, 2) + AIR.D[11] * pow(dTdb, 3) \
                   + AIR.D[12] * math.log(dTdb)

        return math.exp(dLNP) * 0.001

     
    def FTdb__XRh(dX, dRh, dP):
        J = 1
        T1 = 281.0
        DI = AIR.FX__TdbRh(T1, dRh, dP)
        T2 = 323.0
        DII = AIR.FX__TdbRh(T2, dRh, dP)
        DIII = DII  # liu 20220320

        while J <= 40:
            T3 = T2 - (T2 - T1) / (DII - DI) * (DII - dX)
            DIII = AIR.FX__TdbRh(T3, dRh, dP)

            if (DIII / dX - 1.0) >= 1e-5 or (DIII / dX - 1.0) < -1e-5:
                T1 = T2
                T2 = T3
                DI = DII
                DII = DIII
            else:
                break

            J += 1

        TT = T3
        return TT

     
    def FMu__Tdb(dTdb):
        return 17.268e-6 * (dTdb / 273.15) ** 0.7

     
    def FLambda__Tdb(Tdb):
        return 2.4066e-2 * (Tdb / 273.15) ** 0.9

    def ResetFlags(self):
        self.flgP = False
        self.flgTdb = False
        self.flgTwb = False
        self.flgTdp = False
        self.flgRh = False
        self.flgX = False
        self.flgH = False
        self.flgS = False
        self.flgV = False
        self.flgMu = False
        self.flgLambda = False

    def getProp(self,unit):
        self.flgTdb = False
        self.flgTwb = False
        self.flgTdp = False
        self.flgRh = False
        self.flgH = False
        self.flgX = False
        self.flgS = False
        self.flgV = False
        self.flgLambda = False
        self.flgDen = False
        if unit =='k' or unit == 'K':
            prop = {
                'dry-bulb temperature(k)': self.dTdb,
                'wet-bulb temperature(k)': self.dTwb,
                'dew-point temperature(k)': self.dTdp,
                'relative humidity(0~1)': self.dRh,
                'specific enthalpy(kj/kg)': self.dH,
                'absolute humidity(kg/kg)': self.dX,
                'specific entropy(kj/kg/k)': self.dS,
                'specific volume(m3/kg)': self.dV,
                'kinematic viscosity(m2/s)': self.dMu,
                'thermal Conductivity(W/m/K)': self.dLambda,
                'Density(kg/m3)': self.Den
            }
            return prop

        if unit =='c' or unit == 'C':
            prop = {
                'dry-bulb temperature(c)': self.dTdb - 273.15,
                'wet-bulb temperature(c)': self.dTwb - 273.15,
                'dew-point temperature(c)': self.dTdp - 273.15,
                'relative humidity(0~1)': self.dRh,
                'specific enthalpy(kj/kg)': self.dH,
                'absolute humidity(kg/kg)': self.dX,
                'specific entropy(kj/kg/k)': self.dS,
                'specific volume(m3/kg)': self.dV,
                'Kinematic viscosity(m2/s)': self.dMu,
                'Thermal Conductivity(W/m/c)': self.dLambda,
                'Density(kg/m3)': self.Den
            }
            return prop


    def setProp(self,dP,unit,**kwargs):
        if unit == 'k' or unit == 'K':
            self.dP = dP
            if 'dTdb' in kwargs:
                self.dTdb = kwargs.get('dTdb')
                self.flgTdb = True
            if 'dTwb' in kwargs:
                self.dTwb = kwargs.get('dTwb')
                self.flgTwb = True
            if 'dTdp' in kwargs:
                self.dTdp = kwargs.get('dTdp')
                self.flgTdp = True
            if 'dRh' in kwargs:
                self.dRh = kwargs.get('dRh')
                self.flgRh = True
            if 'dH' in kwargs:
                self.dH = kwargs.get('dH')
                self.flgH = True
            if 'dX' in kwargs:
                self.dH = kwargs.get('dX')
                self.flgX = True
            if 'dS' in kwargs:
                self.dS = kwargs.get('dS')
                self.flgS = True
            if 'dV' in kwargs:
                self.dV = kwargs.get('dV')
                self.flgV = True
            if 'dMu' in kwargs:
                self.dMu = kwargs.get('dMu')
                self.flgMu = True
            if 'dLambda' in kwargs:
                self.dLambda = kwargs.get('dLambda')
                self.flgLambda = True
        if unit == 'c' or unit == 'C':
            self.dP = dP
            if 'dTdb' in kwargs:
                self.dTdb = kwargs.get('dTdb') + 273.15
                self.flgTdb = True
            if 'dTwb' in kwargs:
                self.dTwb = kwargs.get('dTwb') + 273.15
                self.flgTwb = True
            if 'dTdp' in kwargs:
                self.dTdp = kwargs.get('dTdp') + 273.15
                self.flgTdp = True
            if 'dRh' in kwargs:
                self.dRh = kwargs.get('dRh')
                self.flgRh = True
            if 'dH' in kwargs:
                self.dH = kwargs.get('dH')
                self.flgH = True
            if 'dX' in kwargs:
                self.dH = kwargs.get('dX')
                self.flgX = True
            if 'dS' in kwargs:
                self.dS = kwargs.get('dS')
                self.flgS = True
            if 'dV' in kwargs:
                self.dV = kwargs.get('dV')
                self.flgV = True
            if 'dMu' in kwargs:
                self.dMu = kwargs.get('dMu')
                self.flgMu = True
            if 'dLambda' in kwargs:
                self.dLambda = kwargs.get('dLambda')
                self.flgLambda = True

    def updateData(self):
        if self.flgTdb:
            if self.flgTwb:
                self.dTdp = AIR.FTdp__TdbTwb(self.dTdb, self.dTwb, self.dP)
                self.flgTdp = True
                self.dRh = AIR.FRh__TdbTwb(self.dTdb, self.dTwb, self.dP)
                self.flgRh = True
                self.dH = AIR.FH__TdbTwb(self.dTdb, self.dTwb, self.dP)
                self.flgH = True
                self.dX = AIR.FX__TdbTwb(self.dTdb, self.dTwb, self.dP)
                self.flgX = True
                self.dS = AIR.FS__TdbTwb(self.dTdb, self.dTwb, self.dP)
                self.flgS = True
                self.dV = AIR.FV__TdbTwb(self.dTdb, self.dTwb, self.dP)
                self.flgV = True
                self.dMu = AIR.FMu__Tdb(self.dTdb)
                self.flgMu = True
                self.dLambda = AIR.FLambda__Tdb(self.dTdb)
                self.flgLambda = True
            elif self.flgTdp:
                self.dTwb = AIR.FTwb__TdbTdp(self.dTdb, self.dTdp, self.dP)
                self.flgTwb = True
                self.dRh = AIR.FRh__TdbTdp(self.dTdb, self.dTdp, self.dP)
                self.flgRh = True
                self.dH = AIR.FH__TdbTdp(self.dTdb, self.dTdp, self.dP)
                self.flgH = True
                self.dX = AIR.FX__TdbTdp(self.dTdb, self.dTdp, self.dP)
                self.flgX = True
                self.dS = AIR.FS__TdbTdp(self.dTdb, self.dTdp, self.dP)
                self.flgS = True
                self.dV = AIR.FV__TdbTdp(self.dTdb, self.dTdp, self.dP)
                self.flgV = True
                self.dMu = AIR.FMu__Tdb(self.dTdb)
                self.flgMu = True
                self.dLambda = AIR.FLambda__Tdb(self.dTdb)
                self.flgLambda = True
            elif self.flgRh:
                self.dTwb = AIR.FTwb__TdbRh(self.dTdb, self.dRh, self.dP)
                self.flgTwb = True
                self.dTdp = AIR.FTdp__TdbRh(self.dTdb, self.dRh, self.dP)
                self.flgTdp = True
                self.dH = AIR.FH__TdbRh(self.dTdb, self.dRh, self.dP)
                self.flgH = True
                self.dX = AIR.FX__TdbRh(self.dTdb, self.dRh, self.dP)
                self.flgX = True
                self.dS = AIR.FS__TdbRh(self.dTdb, self.dRh, self.dP)
                self.flgS = True
                self.dV = AIR.FV__TdbRh(self.dTdb, self.dRh, self.dP)
                self.flgV = True
                self.dMu = AIR.FMu__Tdb(self.dTdb)
                self.flgMu = True
                self.dLambda = AIR.FLambda__Tdb(self.dTdb)
                self.flgLambda = True
            elif self.flgH:
                self.dTwb = AIR.FTwb__TdbH(self.dTdb, self.dH, self.dP)
                self.flgTwb = True
                self.dTdp = AIR.FTdp__TdbH(self.dTdb, self.dH, self.dP)
                self.flgTdp = True
                self.dRh = AIR.FRh__TdbH(self.dTdb, self.dH, self.dP)
                self.flgRh = True
                self.dX = AIR.FX__TdbH(self.dTdb, self.dH, self.dP)
                self.flgX = True
                self.dS = AIR.FS__TdbH(self.dTdb, self.dH, self.dP)
                self.flgS = True
                self.dV = AIR.FV__TdbH(self.dTdb, self.dH, self.dP)
                self.flgV = True
                self.dMu = AIR.FMu__Tdb(self.dTdb)
                self.flgMu = True
                self.dLambda = AIR.FLambda__Tdb(self.dTdb)
                self.flgLambda = True
            else:
                # If there is no other "true" flag, X is assumed to be constant
                self.dTwb = AIR.FTwb__TdbX(self.dTdb, self.dX, self.dP)
                self.flgTwb = True
                self.dTdp = AIR.FTdp__TdbX(self.dTdb, self.dX, self.dP)
                self.flgTdp = True
                self.dRh = AIR.FRh__TdbX(self.dTdb, self.dX, self.dP)
                self.flgX = True
                self.dH = AIR.FH__TdbX(self.dTdb, self.dX, self.dP)
                self.flgH = True
                self.dS = AIR.FS__TdbX(self.dTdb, self.dX, self.dP)
                self.flgS = True
                self.dV = AIR.FV__TdbX(self.dTdb, self.dX, self.dP)
                self.flgV = True
                self.dMu = AIR.FMu__Tdb(self.dTdb)
                self.flgMu = True
                self.dLambda = AIR.FLambda__Tdb(self.dTdb)
                self.flgLambda = True
        elif self.flgH and self.flgX:
            self.dTdb = AIR.FTdb__HX(self.dH, self.dX, self.dP)
            self.flgTdb = True
            self.dTwb = AIR.FTwb__HX(self.dH, self.dX, self.dP)
            self.flgTwb = True
            self.dTdp = AIR.FTdp__HX(self.dH, self.dX, self.dP)
            self.flgTdp = True
            self.dRh = AIR.FRh__HX(self.dH, self.dX, self.dP)
            self.flgRh = True
            self.dS = AIR.FS__HX(self.dH, self.dX, self.dP)
            self.flgS = True
            self.dV = AIR.FV__HX(self.dH, self.dX, self.dP)
            self.flgV = True
            self.dMu = AIR.FMu__Tdb(self.dTdb)
            self.flgMu = True
            self.dLambda = AIR.FLambda__Tdb(self.dTdb)
            self.flgLambda = True
        elif self.flgX and self.flgRh:
            self.dTdb = AIR.FTdb__XRh(self.dX, self.dRh, self.dP)
            self.flgTdb = True
            self.dTwb = AIR.FTwb__TdbX(self.dTdb, self.dX, self.dP)
            self.flgTwb = True
            self.dTdp = AIR.FTdp__TdbX(self.dTdb, self.dX, self.dP)
            self.flgTdp = True
            self.dRh = AIR.FRh__TdbX(self.dTdb, self.dX, self.dP)
            self.flgX = True
            self.dH = AIR.FH__TdbX(self.dTdb, self.dX, self.dP)
            self.flgH = True
            self.dS = AIR.FS__TdbX(self.dTdb, self.dX, self.dP)
            self.flgS = True
            self.dV = AIR.FV__TdbX(self.dTdb, self.dX, self.dP)
            self.flgV = True
            self.dMu = AIR.FMu__Tdb(self.dTdb)
            self.flgMu = True
            self.dLambda = AIR.FLambda__Tdb(self.dTdb)
            self.flgLambda = True

        self.Den = AIR.FD__TdbTwb(self.dTdb, self.dTwb, self.dP)
        self.flgDen = True
        self.ResetFlags()

class R134A:
    pass #TODO


if __name__ == "__main__":
    # air = AIR(dTdb = 50,dRh= 0.20,dP = 101.325,unit = 'c')
    # air.updateData()
    # prop = air.getProp('c')
    # for key in prop:
    #     print(f"{key} \t:{prop[key]}")
    # # print(air.dH)
    #
    # air.setProp(dTdb=50, dRh= 0.30, dP=80.325, unit='c')
    # air.updateData()
    # prop = air.getProp('c')
    # for key in prop:
    #     print(f"{key} \t:{prop[key]}")
    # # print(air.dH)
    #
    # air = AIR(dP=80.325, unit='c', dTdb=50, dRh= 0.40)
    # air.updateData()
    # prop = air.getProp(unit='c')
    # for key in prop:
    #     print(f"{key}:{prop[key]}")
    #
    # air.setProp(dP=80.325, unit='c', dTdb=50, dRh= 1)
    # air.updateData()
    # prop = air.getProp(unit='c')
    # for key in prop:
    #     print(f"{key}:{prop[key]}")

    air = AIR(dP=101.325, unit='c',  dTdb=-3.4983, dRh=0.5)
    air.updateData()
    prop = air.getProp(unit='c')
    for key in prop:
        print(f"{key}:{prop[key]}")

    # print("----------------------")
    # air = AIR(dP=101.325, unit='c', dX=0.017872256, dRh=1)
    # air.updateData()
    # prop = air.getProp(unit='c')
    # for key in prop:
    #     print(f"{key}:{prop[key]}")
    #
    # print("----------------------")
    # air = AIR(dP=101.325, unit='c', dX=0.01416923,dH = 84.10859848)
    # air.updateData()
    # prop = air.getProp(unit='c')
    # for key in prop:
    #     print(f"{key}:{prop[key]}")


    # air = AIR(dP=101.325, unit='c',  dTdb=30, dRh=0.8)
    # air.updateData()
    # prop = air.getProp(unit='c')
    # for key in prop:
    #     print(f"{key}:{prop[key]}")
    #
    #
    # print("----------------------")
    # air = AIR(dP=101.325, unit='c', dX=0.021575-0.0001736, dRh=1)
    # air.updateData()
    # prop = air.getProp(unit='c')
    # for key in prop:
    #     print(f"{key}:{prop[key]}")
