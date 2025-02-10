"use client"
import Image from 'next/image'
import Link from 'next/link'


import TituloDoMenu from "./titulo-menu"
import ItemDoMenu from "./item-menu"
import Divisoria from "./divisoria"
import CampoDeTexto from "./campo-texto"
import BotaoDeSalvar from "./botao-salvar"

import Gordao from "../../../../public/positivo.png"
import styles from './menu.module.css'

import {POST} from '../_lib/actions'
import {GET} from '../_lib/actions'

import { useEffect, useState } from 'react';

export function Menu(props){
    const [valor, setValor] = useState("");
    const [erro, setErro] = useState("");
    const [sucesso, setSucesso] = useState("");
    const [isActive, setIsActive] = useState(false);

    useEffect(() => {
        async function checkBackendConnection() {
            try {
                // Realiza um get para verificar se o backend está conectado, não da para ser um post pois não temos uma url para testar
                await GET('/fotos');
                setErro("");
            } catch (error) {
                setErro("Erro ao conectar ao servidor. Verifique a conexão.");
            }
        }
        checkBackendConnection();
    }, []);


    function mudaValor(novoValor){
        setValor(novoValor);
        console.log("mudaValor", novoValor);
        setIsActive(true);
    }

    async function postFoto() {
        setErro('');
        setSucesso('');
        
        if (valor){
            console.log("valor", valor);
            let url = {
                url: valor 
            }
            setValor('');
            setIsActive(false);

            try {
                await POST('/fotos', url);
                setSucesso("Foto cadastrada com sucesso!");
            } catch (error) {
                setErro(error.message)
            }
        }
    }

    return(
        <>
        <div className={styles.menu}>
            <Link href = "/">
            <Image className={styles.gordao} src={Gordao} alt="gordao imagem" />
            </Link>
            <ItemDoMenu uri="/cadastro-de-maquina">Cadastro de maquina</ItemDoMenu>
            <ItemDoMenu uri="/cadastro-de-exercicio">Cadastro de exercício</ItemDoMenu>
            <ItemDoMenu uri="/cadastro-de-serie">Cadastro de série</ItemDoMenu>
            <ItemDoMenu uri="/cadastro-de-treino">Cadastro de treino</ItemDoMenu>

            <Divisoria/>
            <TituloDoMenu >DEBUG: Cadastro de foto de máquina</TituloDoMenu>
            <CampoDeTexto 
                name="" 
                value={valor} 
                onChange={mudaValor}
                />

            <BotaoDeSalvar onClick={postFoto} isActive={isActive}/>

            {erro && (
                <div className={styles.mensagemErro}> {erro} </div>  
            )}

            {sucesso && (
                <div className={styles.mensagemSucesso}> {sucesso} </div>  
            )}

        </div>
        </>
    ); 
}