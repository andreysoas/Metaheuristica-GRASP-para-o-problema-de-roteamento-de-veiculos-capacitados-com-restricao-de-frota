import numpy as np
import itertools
import numpy.random as rd
from math import sqrt,ceil
from random import sample
from copy import deepcopy
from math import floor
import time

"""#Objeto que encapsulará a distribuição de alfa"""

class alfaDistribuicao:

  def __init__(self):
    self.alfaList = [0.7,0.75,0.8,0.9,0.95]
    self.probAlfa = [1/5,1/5,1/5,1/5,1/5]
    self.sumAlfa = {0.7:[0,0],0.75:[0,0],0.8:[0,0],0.9:[0,0],0.95:[0,0]}

  def attAlfa(self,alfa,valor):
    self.sumAlfa[alfa][0] = self.sumAlfa[alfa][0]+valor
    self.sumAlfa[alfa][1] = self.sumAlfa[alfa][1]+1
    
  def recalcularDist(self,currentMin):

    if [0,0] in self.sumAlfa.values(): return False

    A = {0.6:None,0.7:None,0.8:None,0.9:None,0.95:None} #Média dos valores para Alfa

    q_key = []

    for Key in self.alfaList:
      A[Key] = self.sumAlfa[Key][0] / self.sumAlfa[Key][1]
      self.sumAlfa[Key] = [0,0]

    for Key in self.alfaList:
      q_key.append((currentMin/A[Key])**10)
    
    for i in range(5):
      self.probAlfa[i] = q_key[i]/sum(q_key)
    
    return True
  
  def retornarAlfa(self):
    return rd.choice(self.alfaList,size=1,p=self.probAlfa)[0]

"""#Função principal"""

def cvrpSolution(path:str):
  ''' 
  parâmetros e variáveis do problema
  '''
  demandas = []
  coordenadas = []
  capacidade = None
  startPoint = None
  qtdEntidades = None #receber na entrada
  qtdVeiculos = None #None  #receber na entrada
  currentMin = 999999
  opt = None
  numIter = 500

  objDist = alfaDistribuicao()

  '''
  carregando parâmetros, obtendo solução inicial e calculando seu valor
  '''

  capacidade,startPoint,qtdEntidades,qtdVeiculos,opt = carregarDados(path,demandas,coordenadas,startPoint)

  matrizAdj = distMatriz(coordenadas,qtdEntidades)

  min = None

  for i in range(numIter):

    solAtual = VMPR(qtdEntidades,capacidade,startPoint,
                    qtdVeiculos,matrizAdj,demandas,currentMin,objDist,i+1)

    solVal = solValue(solAtual, matrizAdj,capacidade)

    #print("valor antes: ",solVal)

    if i==0:
      min  = deepcopy(solAtual)
      currentMin = solVal

    solVal = VND(solAtual,solVal,matrizAdj,capacidade,qtdVeiculos,demandas)
    #print('valor depois: ',solVal)
    if solVal < currentMin:
      currentMin = solVal
      min = deepcopy(solAtual)

  '''
    print('->',solAtual,' | ',solVal)
  '''
  return solAtual,currentMin,opt

"""#Carregamento dos dados"""

def carregarDados(path:str,demandas,coordenadas,startPoint):

  arq = open(path,'r')

  takeNodes = False
  takeDemands = False
  takeDepot = False
  contEntidades = None
  contVehicles = None
  optimal = None
  capacidade = None

  section = 1

  for line in arq:

    
    if section == 2:

      if takeNodes: #--------------------

        if len(coordenadas) == contEntidades:
          takeNodes = False
          takeDemands = True
        else:
          seps = line.split(' ')
          coordenadas.append([int(seps[1]),int(seps[2]),int(seps[3][:-1])])


      elif takeDemands: #----------------

        if len(demandas) == contEntidades:
          takeDemands = False
          takeDepot = True

        else:
          demandas.append(int(line.split(' ')[1][:]))


      elif takeDepot: #------------------

        startPoint = int(line[1:-1])

        takeDepot = False

        section+=1


    
    if section == 1:

      if 'COMMENT' in line:
        seps = line.split(': ')
        contVehicles = int(seps[2].split(',')[0])
        optimal = int(seps[3][:-2]) #\n
      
      elif 'DIMENSION' in line:
        contEntidades = int(line.split(': ')[1][:-1])
      
      elif 'CAPACITY' in line:
        capacidade = int(line.split(': ')[1][:-1])
      
      elif 'NODE' in line:
        takeNodes = True
        section+=1



  arq.close()

  return capacidade,startPoint,contEntidades,contVehicles,optimal

"""#Funções auxiliares"""

def distMatriz(coordenadas,qtdEntidades):
  
  matrizAdj = np.ones((qtdEntidades,qtdEntidades))
  m = 0

  for i in coordenadas:
    n=0
    for j in coordenadas:
      if m!=n:
        matrizAdj[m][n] = round(sqrt((i[2]-j[2])**2 + (i[1]-j[1])**2))
      else:
        matrizAdj[m][n] = 9999
      n+=1
    m+=1

  return matrizAdj

def solValue(solucao:list,matrizAdj:np.ndarray,capacidade:int): #considerando solução vazia
  valor = 0

  for i in solucao:
    valor+=calcOneroute(i[1],matrizAdj)
    
  return valor

def calcOneroute(route,matrizAdj):
  valor = 0
  
  for i in range(1,len(route)):
    valor += matrizAdj[route[i-1]-1][route[i]-1]
  valor+=matrizAdj[route[-1]-1][0]
  
  return valor

"""#VND"""

def N1(solAtual:list,qtdVeiculos:int,capacidade:int,matrizAdj):

  melhorou = False
  newSol = deepcopy(solAtual)


  for i in range(qtdVeiculos):

    if len(newSol[i][1]) >= 3: #2 ou mais clientes visitados
      valor1 = calcOneroute(newSol[i][1],matrizAdj) #valor da rota sem a mudança'''
      for j in range(1,len(newSol[i][1])):
        for k in range(j+1,len(newSol[i][1])):

          newSol[i][1][j],newSol[i][1][k] = newSol[i][1][k],newSol[i][1][j]

          valor2 = calcOneroute(newSol[i][1],matrizAdj) #valor da rota com a mudança'''
          #print('valor1: ',valor1)
          #print('valor2: ',valor2)
          if valor2 < valor1:                               #se houve melhora
            melhorou=True
          else:
            newSol[i][1][j],newSol[i][1][k] = newSol[i][1][k],newSol[i][1][j] 

  return melhorou

def N2(solAtual:list,qtdVeiculos:int,capacidade:int,matrizAdj,demandas,atualMin): #Troca de um elemento entre rotas
 
  solOpt = [deepcopy(solAtual)]
  newSol = deepcopy(solAtual)
  melhorou = False


  for i in range(qtdVeiculos-1):
      for j in range(1,len(newSol[i][1])):
        for m in range(i+1,qtdVeiculos):
          for n in range(1,len(newSol[m][1])):  #j e n

            capacidade1 = newSol[i][0] + demandas[newSol[i][1][j]-1] - demandas[newSol[m][1][n]-1]
            capacidade2 = newSol[m][0] + demandas[newSol[m][1][n]-1] - demandas[newSol[i][1][j]-1]
                                                                          #checagem da viabilidade da alteração
            if capacidade1>=0 and capacidade2>=0:

              newSol[m][1][n],newSol[i][1][j] = newSol[i][1][j],newSol[m][1][n]

              newSol[i][0],capacidade1 = capacidade1,newSol[i][0]
              newSol[m][0],capacidade2 = capacidade2,newSol[m][0]
                                                                          #Checagem para decidir se a alteração diminui o valor da solução
              valor = solValue(newSol,matrizAdj,capacidade)

              if valor < atualMin: #atual Mínimo
                solOpt[0] = deepcopy(newSol)
                atualMin = valor
                melhorou = True

              newSol[m][1][n],newSol[i][1][j] = newSol[i][1][j],newSol[m][1][n]
              newSol[i][0] = capacidade1
              newSol[m][0] = capacidade2

  for i in range(qtdVeiculos):
    solAtual[i][0] = solOpt[0][i][0]
    for j in range(len(solAtual[i][1])):
      solAtual[i][1][j] = solOpt[0][i][1][j]

  return melhorou

def N4(solAtual,qtdVeiculos,demandas,capacidade,currentMin,matrizAdj): #destacar rotas destacar 1 elemento
  newSol = deepcopy(solAtual)

  for k in range(5):
    rotas = list(np.arange(0,qtdVeiculos))
    rotasDisponiveis = []
    for i in range(qtdVeiculos):
      if len(newSol[i][1]) > 1:
        rotasDisponiveis.append(i)

    rotaDestacada = sample(rotasDisponiveis,1)[0]
    rotas.remove(rotaDestacada)
    elementosParaExcluir = []

    for j in range(1,len(newSol[rotaDestacada][1])):
      elemento = newSol[rotaDestacada][1][j]
      demandaElemento = demandas[elemento -1]
      for i in rotas:
        if demandaElemento <= newSol[i][0]:
          
          newSol[i][0] -= demandaElemento

          newSol[i][1].append(elemento)

          elementosParaExcluir.append(elemento)

          break
    #print(elementosParaExcluir)
    for i in elementosParaExcluir:
      newSol[rotaDestacada][0] += demandas[i -1]
      newSol[rotaDestacada][1].remove(i)

  valorMudanca = solValue(newSol,matrizAdj,capacidade)

  if valorMudanca < currentMin:
    for i in range(qtdVeiculos):
      solAtual[i][0] = newSol[i][0]
      solAtual[i][1]=deepcopy(newSol[i][1])

  return

def N5(solAtual,qtdVeiculos,matrizAdj):#2-opt

  newSol = deepcopy(solAtual)
 
  for i in range(qtdVeiculos):
    if len(newSol[i][1])>=4:
      newSol[i][1] = reverse(newSol[i][1],matrizAdj)

  for i in range(qtdVeiculos):
    solAtual[i][0] = newSol[i][0]
    solAtual[i][1] = deepcopy(newSol[i][1])
  
  return

def reverse(rota,matrizAdj):
  rangeMin = 2
  rangeMax = len(rota)-1 -1 #parte destacada fora o início

  rotaSave = deepcopy(rota)
  valAtual = calcOneroute(rota,matrizAdj)

  for i in (rangeMin,rangeMax): #[1,2,3,4] -> len = 4 -> 3-1
    
    for j in (1,len(rota)-i):
      copia = rota[j:j+i]

      for k in range(i):
        rota[j+k] = copia[i-1-k]
    
      valNova = calcOneroute(rota,matrizAdj)

      if valNova < valAtual:
        return rota

      for k in range(len(rota)):
        rota[k] = rotaSave[k]

  return rota

def VND(solAtual:list,currentMin,matrizAdj,capacidade,qtdVeiculos,demandas): # ( Usando cyclic neighborhood change step)

  atualMin = currentMin
  for i in range(10):
    N1(solAtual,qtdVeiculos,capacidade,matrizAdj)
    atualMin = solValue(solAtual,matrizAdj,capacidade)

    N2(solAtual,qtdVeiculos,capacidade,matrizAdj,demandas,atualMin)
    atualMin = solValue(solAtual,matrizAdj,capacidade)
    
    N4(solAtual,qtdVeiculos,demandas,capacidade,atualMin,matrizAdj)
    atualMin = solValue(solAtual,matrizAdj,capacidade)

    N5(solAtual,qtdVeiculos,matrizAdj)
    atualMin = solValue(solAtual,matrizAdj,capacidade)

  return atualMin

"""#VMP random"""

def VMPR(qtdEntidades,capacidade,startPoint,qtdVeiculos,
         matrizAdj,demandas,currentMin,objDist,iter):
  
  while True:
    alfa = objDist.retornarAlfa()

    solObtida = solbyGuloso(qtdEntidades,capacidade,startPoint,
                            qtdVeiculos,matrizAdj,demandas,alfa)

    if solObtida[0]:
      objDist.attAlfa(alfa,solValue(solObtida[1],matrizAdj,capacidade))

      if iter%50==0: 
        if objDist.recalcularDist(currentMin): #Atualizando a cada 50 iterações(se houver ocorrências de todos os alfas)
          print('Atualização das probabilidades de alfa:\n',objDist.probAlfa)
      break

  return solObtida[1]



def solbyGuloso(qtdEntidades,capacidade,startPoint,
                qtdVeiculos,matrizAdj,demandas,alfa): 

  entidades = np.arange(1,qtdEntidades+1,dtype=int)

  solInicial = [[capacidade,[]] for i in range(qtdVeiculos)]
  
  setDisponiveis = [i for i in range(1,qtdEntidades+1)]
  setDisponiveis.remove(startPoint)
  tamSet = qtdEntidades
  disp = None

  for i in range(qtdVeiculos):
    setDisponiveis.append(startPoint)
    disp = VMP(startPoint,solInicial[i],matrizAdj,
               setDisponiveis,demandas,alfa)
 
  
  if len(disp)>0:
    return (False,-1)
  else:
    return (True,solInicial)#,solValue(solInicial)


def VMP(atualV,route,matrizAdj,setDisponiveis,demandas,alfa):
 
  route[0] -= demandas[atualV-1]
  route[1].append(atualV)
  setDisponiveis.remove(atualV)

  change = False

  LC = []
  valorMinimo = 9999

  for i in setDisponiveis:
    if demandas[i-1] <= route[0]:

      if matrizAdj[atualV-1][i-1] < valorMinimo:
        valorMinimo = matrizAdj[atualV-1][i-1]  
      
      LC.append([matrizAdj[atualV-1][i-1],i]) #distancia,vertice
      
      change = True

  if change:

    limSuperior = valorMinimo/alfa

    LCR = []

    for i in range(len(LC)):
      if LC[i][0] <= limSuperior:
        LCR.append(LC[i])

    chosenOne = LCR[rd.randint(0,len(LCR))]

    atualV = chosenOne[1]

  else: 
    return setDisponiveis

  return VMP(atualV,route,matrizAdj,setDisponiveis,demandas,alfa)

"""#Main"""

def main():
  
  path = input('path do arquivo txt: ')
  
  try:
    ini = time.time()
    result = cvrpSolution(path)
    fim = time.time()
  except FileNotFoundError:
    print('Diretório incorreto!')
    return
  
  print('\nResultados :',end='\n')
  print('tempo( em segundos ): ',fim-ini)
  print('Desvio com relação ao ótimo: ',round(((result[1] - result[2])/result[2]),4)*100,'%')
  print('Valor de solução: ',result[1])
  print('Solução: ',result[0])
  return

if __name__ == '__main__':
  main()
