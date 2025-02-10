'use client';

import BotaoSalvar from './_components/botao-salvar';
import CampoTexto from './_components/campo-texto';
import CardTreino from './_components/card-treino'
import Divisoria from './_components/divisoria';
import Titulo from './_components/titulo';

import styles from './page.module.css';
import { useEffect, useRef, useState } from 'react';
import { GET , POST } from './_lib/actions';


export default function PaginaInicial() {
    const [textoNotif, setTextoNotif] = useState("")
    const [treinos,setTreinos] = useState([])
    const [erro,setErro] = useState()
    
    
    useEffect(()=>{
        async function fetchData() {
            try{setTreinos(await GET('/treinos'))}
            catch(e){setErro("Erro ao conectar ao servidor. Erro: " + e.message);}
        }
        fetchData()
    },[])
    
    
    const input = useRef(null)
    useEffect(()=>{
        const handler = (e)=>{
            if (e.key === 'Enter' && e.target == input.current.firstChild.firstChild){
                salvar()
            }
        }
        window.addEventListener('keydown', handler)
        return () =>{window.removeEventListener('keydown', handler)}
    },[textoNotif])
    
    async function salvar() {
        if (textoNotif){
            let notificacao = {
                'mensagem':textoNotif
            }
            setTextoNotif("")
            try {
                await POST('/notificacoes', notificacao);
            } catch (error) {
                setErro("Erro de salvamento: " + error.message);
            }
        }
    }''

    function updateTextNotif(txt){
        if (txt != " "){
            setTextoNotif(txt);
        }
    }
    
    return (
        <>
            {erro && (
                    <div onClick={()=>{setErro('')}} id={styles.mensagemErro}><p>{erro} <span>[X]</span></p></div>  
                )}

            <div id={styles.containerInicial}>
                <Titulo>Próximos Treinos</Titulo>
                <div id={styles.listaCards}>
                    { treinos.length != 0 ?
                            treinos.map((card) => <CardTreino
                                data={card.dia.split('-').reverse().join('/')}
                                hora={card.hora.slice(0,-3)}
                                numero={card.series.length}
                                key={card.id}/>) :
                            (<p>Não há treinos disponíveis</p>)}
                </div>
                <div id={styles.divisoria}>
                    <Divisoria/>
                </div>

                <Titulo>DEBUG: Cadastro de notificação</Titulo>
                <div ref={input}>
                    <CampoTexto
                        value={textoNotif}
                        onChange={updateTextNotif}
                    />
                </div>

                <div id={styles.botaoSalvarInicial}>
                    <BotaoSalvar onClick={salvar} isActive={textoNotif} />
                </div>

            </div>
        </>
    );
}