"use client"
import Image from 'next/image'
import Link from 'next/link'
import { useState, useRef, useEffect} from 'react';

import BotaoDeMenu from "./botao-menu";
import BotaoDeNotificacoes from "./botao-notificacoes";
import PopUp from './pop-up-header';
import {Menu} from "./menu";

import styles from './header.module.css';
import Gordao from "../../../../public/positivo.png";

import {GET} from '../_lib/actions'

export default function Header() {
    const [menuVisivel, setMenuVisivel] = useState(false);
    const [popUpVisivel, setPopUpVisivel] = useState(false);
    const [notificacoes, setNotificacoes] = useState([]);
    const menuRef = useRef(null);

    function clica_menu(){
        setMenuVisivel(!menuVisivel);
    }

    function clica_notificacao(){
        setPopUpVisivel(!popUpVisivel);
    }

    useEffect(() => {
        function handleClickOutside(event) {
            if (menuRef.current && !menuRef.current.contains(event.target)) {
                setMenuVisivel(false);
            }
        }
        document.addEventListener("mousedown", handleClickOutside);
        return () => {
            document.removeEventListener("mousedown", handleClickOutside);
        };
    }, []);

    useEffect(() => {
        async function get_notificacoes() {
            try{
                const data = await GET('/notificacoes');
                setNotificacoes(data); 
            }
            catch (error){
                console.log("Erro ao conectar ao servidor. Erro: " + error.message);
            }
        }
        get_notificacoes(); 
    }, []);

    return(
        <> 
            <div className={styles.linha}>
                <BotaoDeMenu
                    onClick={clica_menu}
                />
                <Link href='/'>
                    <Image className={styles.gordao} src={Gordao} alt="gordao imagem" />
                </Link>
                <BotaoDeNotificacoes
                    number={notificacoes.length}
                    onClick={clica_notificacao}
                />
            </div>

            {popUpVisivel && <div>
                <PopUp lista={notificacoes} />
            </div>}

            <div>
                {menuVisivel && 
                <div ref={menuRef}>
                    <Menu />
                </div>}
            </div>
        </>
    )
}

